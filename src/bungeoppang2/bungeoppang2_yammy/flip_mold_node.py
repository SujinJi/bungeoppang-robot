"""붕어빵 틀 뒤집기

순서: 홈 → 손잡이 뒤에서 대기 → 손잡이 잡기 → 위로 들기 → J6 180° 회전 → 원래 높이로 내리기 → 놓기 → 빠지기 → 홈

실행:
  에뮬레이터:  ros2 run bungeoppang flip_mold
  실물:        ros2 run bungeoppang flip_mold --real        (단계마다 Enter로 확인)
  실물 연속:   ros2 run bungeoppang flip_mold --real --no-step
"""
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
import DR_init

from bungeoppang2_yammy import workcell as W

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    real = "--real" in sys.argv
    step = real and "--no-step" not in sys.argv     # 실물에서는 기본으로 단계별 확인
    scale = W.SPEED_SCALE_REAL if real else 1.0

    rclpy.init(args=args)
    node = rclpy.create_node("bungeoppang_flip", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR_ROBOT2는 반드시 DR_init에 노드를 넣은 '다음'에 import
    from DSR_ROBOT2 import (movej, movel, posj, posx, get_current_posx, get_current_posj,
                            set_robot_mode, ROBOT_MODE_AUTONOMOUS)

    # 그리퍼는 별도 노드 + 별도 스레드 (DSR_ROBOT2 노드와 섞이지 않게)
    grip_node = rclpy.create_node("bungeoppang_gripper")
    ex = SingleThreadedExecutor()
    ex.add_node(grip_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    from bungeoppang2_yammy.gripper_ctrl import GripperCtrl
    gripper = GripperCtrl(grip_node, real=real)

    log = node.get_logger()

    def confirm(msg):
        log.info(f"▶ {msg}")
        if step:
            input("   Enter = 실행 / Ctrl+C = 중단 ")

    def tool_z(pose):
        tx, ty, tz = W.tool_tip(pose)
        return ((tx - pose[0]) / W.TOOL_LENGTH, (ty - pose[1]) / W.TOOL_LENGTH,
                (tz - pose[2]) / W.TOOL_LENGTH)

    def along_tool(pose, dist):
        """툴 z 방향(그리퍼가 향하는 방향)으로 dist(mm)만큼 옮긴 자세"""
        zx, zy, zz = tool_z(pose)
        p = list(pose)
        p[0] += dist * zx
        p[1] += dist * zy
        p[2] += dist * zz
        return p

    def move_l(pose, label, vel=None):
        W.check_pose(pose, label)                        # 안전 검사 → 범위 밖이면 여기서 멈춤
        confirm(f"{label}: {[round(v, 1) for v in pose]}")
        v = (vel or W.VEL_L) * scale
        movel(posx(*pose), vel=v, acc=W.ACC_L * scale)

    def current_x():
        pos, _ = get_current_posx()
        return list(pos)

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        # 0) 홈
        confirm("홈 자세로 이동")
        movej(posj(*W.HOME_J), vel=W.VEL_J * scale, acc=W.ACC_J * scale)
        gripper.open()

        # 1) 손잡이 뒤에서 대기 → 2) 잡는 위치로 천천히
        grip = list(W.MOLD_GRIP)
        move_l(along_tool(grip, -W.APPROACH_BACK), "손잡이 뒤 대기")
        move_l(grip, "손잡이 잡는 위치", vel=W.VEL_SLOW_L)

        # 3) 잡기  ★ 조임값은 workcell.py의 MOLD_GRIP_WIDTH / MOLD_GRIP_CURRENT
        confirm("손잡이 잡기")
        gripper.grip_mold()

        # 4) 위로 들기
        up = list(grip)
        up[2] += W.LIFT_HEIGHT
        move_l(up, "위로 들기", vel=W.VEL_SLOW_L)

        # 5) J6 180° 회전 (관절 한계를 넘지 않는 방향을 자동 선택)
        j = list(get_current_posj())
        sign = 1.0 if j[5] + W.FLIP_ANGLE <= W.J6_LIMIT else -1.0
        if abs(j[5] + sign * W.FLIP_ANGLE) > W.J6_LIMIT:
            raise ValueError(f"[안전 정지] J6={j[5]:.0f}°: 어느 방향으로도 {W.FLIP_ANGLE:.0f}° 못 돌림")
        target_j = list(j)
        target_j[5] += sign * W.FLIP_ANGLE
        confirm(f"틀 뒤집기: J6 {j[5]:.0f}° → {target_j[5]:.0f}°")
        movej(posj(*target_j), vel=W.VEL_FLIP * scale, acc=W.VEL_FLIP * scale)

        # 6) 원래 높이로 내리기 (x, y는 그대로, 자세는 뒤집힌 상태)
        down = current_x()
        down[2] -= W.LIFT_HEIGHT
        move_l(down, "원래 자리에 내려놓기", vel=W.VEL_SLOW_L)

        # 7) 놓기 → 손잡이에서 뒤로 빠지기 → 위로
        confirm("그리퍼 열기")
        gripper.open()
        back = along_tool(down, -W.APPROACH_BACK)
        move_l(back, "뒤로 빠지기", vel=W.VEL_SLOW_L)
        back[2] += W.RETREAT_UP
        move_l(back, "위로 빠지기")

        # 8) 홈
        confirm("홈 자세로 복귀")
        movej(posj(*W.HOME_J), vel=W.VEL_J * scale, acc=W.ACC_J * scale)
        log.info("✅ 틀 뒤집기 완료")

    except ValueError as e:
        log.error(str(e))
    except KeyboardInterrupt:
        log.warning("사용자가 중단했어요")
    finally:
        ex.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
