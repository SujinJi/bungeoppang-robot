"""붕어빵 전체 공정 — 오늘 버전

  (서빙 모드) 손님 인식 → ㄱ자 인사 / 주문 인식 → 그리퍼 끄덕(위아래) / 못 알아들음 → 도리도리(좌우)
  1차 반죽: 주전자 손잡이 잡기 → 손잡이 하늘 향하게 들기 → 주둥이를 틀 2/3 지점 →
            25도 기울이기 → 2초 → 수평 복귀 → 든 높이로 제자리 이동 → 내려놓기 → 풀기
  속재료:   소스통 중간을 '들 수 있을 만큼만' 잡기 → 입구 위로 든 채 이동 →
            입구를 틀 쪽(아래)으로 → 틀 위 2cm → 조여서(짜며) 아래→위로 →
            입구 다시 위로 → 제자리
  2차 반죽: 1차와 같고 기울인 채 1초
  (반죽·소스는 먼 판(마주보는 틀)에 — positions.POUR_PLATE)
  뚜껑 닫기: 가까운 판 손잡이를 옆에서 잡고 경첩 호 180→0도 (먼 판 위에 덮음) → 그 자리에서 손만 벌림
  밑면 익기 10초 (손은 그 자리) → 뒤집기: 그 자리에서 겹친 손잡이(4cm)를 잡고 +25cm → 손목(J6) 180도 → -25cm
  → 잡은 채 10초 → 같은 방법으로 되돌리기 (FLIP_BACK) → 뚜껑 열기: 위 손잡이만 다시 잡고 180도 펼치기

값 수정: grip_params.py(힘·폭) / positions.py(위치·각도·시간)

실행:
  단독 1회:    ros2 run bungeoppang2 bake_sequence [팥|슈크림]
  실물:        ros2 run bungeoppang2 bake_sequence --real           (단계마다 Enter)
  주문 대기:   ros2 run bungeoppang2 bake_sequence --serve
    /bungeoppang/start_bake("팥")  → 굽고 /bungeoppang/bake_done 발행
    /bungeoppang/gesture("greet"|"yes"|"no") → 인사/끄덕/도리도리
"""
import math
import queue as _queue
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
import DR_init

from bungeoppang2_yammy import grip_params as G
from bungeoppang2_yammy import positions as P

ROBOT_ID = "dsr01"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = "e0509"


def main(args=None):
    real = "--real" in sys.argv
    step = real and "--no-step" not in sys.argv
    scale = P.SPEED_SCALE_REAL if real else 1.0

    rclpy.init(args=args)
    node = rclpy.create_node("bungeoppang_bake", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    from DSR_ROBOT2 import (movej, movel, posj, posx, get_current_posj, get_current_posx,
                            set_robot_mode, wait, ROBOT_MODE_AUTONOMOUS, DR_MV_MOD_REL)

    grip_node = rclpy.create_node("bungeoppang_gripper")
    ex = SingleThreadedExecutor()
    ex.add_node(grip_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    from bungeoppang2_yammy.gripper_ctrl import GripperCtrl
    gripper = GripperCtrl(grip_node, real=real)
    log = node.get_logger()

    # 총괄(orchestrator) 연동
    done_pub = grip_node.create_publisher(Bool, "/bungeoppang/bake_done", 10)
    held_pub = grip_node.create_publisher(String, "/bungeoppang/held", 10)
    orders = _queue.Queue()
    gestures = _queue.Queue()
    grip_node.create_subscription(String, "/bungeoppang/start_bake",
                                  lambda m: orders.put(m.data), 10)
    grip_node.create_subscription(String, "/bungeoppang/gesture",
                                  lambda m: gestures.put(m.data), 10)

    # ---------------- 공통 도우미 ----------------
    def confirm(msg):
        log.info(f"▶ {msg}")
        if step:
            input("   Enter = 실행 / Ctrl+C = 중단 ")

    def offset_tool(pose, dist):
        """그리퍼가 향하는 방향(틀 쪽)으로 dist만큼 옮긴 자세. 음수 = 뒤로"""
        zx, zy, zz = P.tool_dir(pose)
        q = list(pose)
        q[0] += dist * zx; q[1] += dist * zy; q[2] += dist * zz
        return q

    def move_l(pose, label, vel=None):
        P.check_pose(pose, label)                 # 안전 검사 (positions.py)
        P.check_path(cur(), pose, label)          # 가는 길 중간도 검사
        movel(posx(*pose), vel=(vel or P.VEL_L) * scale, acc=P.ACC_L * scale)
        got = cur()                               # 실제로 도착했는지 확인
        err = math.dist(got[:3], pose[:3])
        if err > P.REACH_TOL:
            raise ValueError(f"[도달 실패] {label}: 목표와 {err:.0f}mm 차이 "
                             f"(목표 {[round(v) for v in pose[:3]]}, 실제 {[round(v) for v in got[:3]]}) "
                             + _joint_hint())

    def _joint_hint():
        """도달 실패 원인 힌트: 관절이 한계(±360°)에 걸렸는지 먼저 확인"""
        try:
            j = list(get_current_posj())
        except Exception:
            return "→ 로봇이 이 자세로 갈 수 없어요. positions.py에서 위치를 조정하세요"
        over = [f"J{i+1}={v:.0f}°" for i, v in enumerate(j) if abs(v) > P.J_WOUND_WARN]
        if over:
            return (f"→ 관절이 한계(±360°) 근처까지 감겨 있어요 ({', '.join(over)}). 위치 문제가 아니에요. "
                    "티치펜던트에서 알람 리셋 후 그 관절을 0° 쪽으로 조그하세요")
        return f"(관절 {[round(v) for v in j]}) → 로봇이 이 자세로 갈 수 없어요. positions.py에서 위치를 조정하세요"

    def unwind_if_wound(label):
        """빈손일 때만 호출: J4가 많이 감겨 있으면 홈(J4=0)으로 갔다 와서 풀기.
        (붓기·짜기 동안 J4가 +360° 가까이 누적돼 다음 직선 이동에서 한계를 넘었음)"""
        j = list(get_current_posj())
        if abs(j[3]) > P.J4_UNWIND_DEG:
            confirm(f"{label}: J4가 {j[3]:.0f}° 감겨 있어 빈손으로 홈에 가서 풀기")
            home()

    def go_rel_j6(deg, vel=None):
        j = list(get_current_posj())
        if abs(j[5] + deg) > P.J6_LIMIT:
            deg = -deg
            if abs(j[5] + deg) > P.J6_LIMIT:
                raise ValueError(f"[안전 정지] J6={j[5]:.0f}°: 회전 불가")
        movej(posj(0, 0, 0, 0, 0, deg), vel=(vel or P.VEL_FLIP) * scale,
              acc=(vel or P.VEL_FLIP) * scale, mod=DR_MV_MOD_REL)
        return deg

    def cur():
        pos, _ = get_current_posx()
        return list(pos)

    def lift_clear():
        """관절 이동(movej) 전에 손끝을 작업대 위 안전 높이로 먼저 올리기.
        movej는 관절을 직선 보간해서 손끝이 어디로 지나갈지 보장이 없어요.
        낮은 위치에서 바로 movej를 하면 팔이 작업대나 물체를 쓸고 지나갈 수 있어서,
        큰 자세 변경 전에는 항상 수직으로 먼저 들어 올려요."""
        try:
            c = cur()
            safe_z = P.TRANSIT_FREE_Z + 50.0
            if c[2] < safe_z:
                c[2] = safe_z
                move_l(c, "자세 바꾸기 전 들어올리기", vel=P.VEL_SLOW_L)
        except Exception:
            pass          # 시작 직후 등 현재 자세를 못 읽으면 그냥 진행

    def home(raise_first=True):
        if raise_first:
            lift_clear()
        movej(posj(*P.HOME_J), vel=P.VEL_J * scale, acc=P.ACC_J * scale)

    def swing_j1(pose):
        """1축(J1)만 돌려 목표 쪽을 바라보게. 높이·팔 모양이 그대로라 손에 든 물체가 안 기울고 안 내려가요."""
        c = cur()
        d = math.degrees(math.atan2(pose[1], pose[0]) - math.atan2(c[1], c[0]))
        d = (d + 180.0) % 360.0 - 180.0
        if abs(d) < 3.0:
            return
        j1 = list(get_current_posj())[0]
        if abs(j1 + d) > P.J1_LIMIT:
            d = d - 360.0 if d > 0 else d + 360.0
        movej(posj(d, 0, 0, 0, 0, 0), vel=P.VEL_J * scale, acc=P.ACC_J * scale, mod=DR_MV_MOD_REL)

    def move_jx(pose, label):
        """큰 이동: ① 목표 높이까지 수직으로 올리고 ② 1축만 돌려 방향 맞추고 ③ 나머지는 직선.
        예전엔 처음부터 관절 보간(movejx)으로 가서 중간에 손목이 꺾이며 비커가 틀 쪽으로 내려갔어요.
        직선으로 못 가는 경우에만 예전 방식(다른 팔 자세 시도)으로 갑니다."""
        P.check_pose(pose, label)
        c = cur()
        if c[2] < pose[2] - 1.0:
            up = list(c); up[2] = pose[2]
            move_l(up, f"{label}: 먼저 수직으로 올리기", vel=P.VEL_SLOW_L)
        swing_j1(pose)
        try:
            move_l(pose, label)
            return
        except ValueError as e:
            log.warning(f"   직선으로 못 감 → 관절 이동으로 재시도 ({e})")
        try:
            from DSR_ROBOT2 import movejx, get_current_solution_space
            c = int(get_current_solution_space())
        except Exception as e:
            log.warning(f"movejx 사용 불가({e}) → 직선 이동")
            movel(posx(*pose), vel=P.VEL_L * scale, acc=P.ACC_L * scale)
            c = None
        if c is not None:
            order = []
            for x in (0, 1, 4, 5, 2, 3, 6, 7):      # 같은 자세 → 손목만 → 어깨 → (마지막) 팔꿈치
                if (c ^ x) not in order:
                    order.append(c ^ x)
            for sol in order:
                movejx(posx(*pose), vel=P.VEL_J * scale, acc=P.ACC_J * scale, sol=sol)
                if math.dist(cur()[:3], pose[:3]) <= P.REACH_TOL:
                    if sol != c:
                        log.info(f"   (팔 자세를 바꿔서 도착: 해 {c} → {sol})")
                    break
        got = cur()
        err = math.dist(got[:3], pose[:3])
        if err > P.REACH_TOL:
            raise ValueError(f"[도달 실패] {label}: 목표와 {err:.0f}mm 차이 "
                             f"(목표 {[round(v) for v in pose[:3]]}, 실제 {[round(v) for v in got[:3]]})")

    def pick(pose, open_w, grip_w, current, label, obj=None, side=0.0, back=None):
        confirm(f"{label} 잡기")
        gripper.move(open_w, wait=0.3)
        if obj and side > 0:
            home()
            appr = offset_tool(pose, -side)
            high = list(appr); high[2] = P.OBJ_TRANSIT_Z
            move_jx(high, f"{label} 접근점 위 (1축 회전)")
            move_l(appr, f"{label} 옆으로 내려가기", vel=P.VEL_SLOW_L)
            move_l(pose, f"{label} 파지점", vel=P.VEL_SLOW_L)
        elif obj:
            home()
            high = list(pose); high[2] = max(pose[2] + P.APPROACH_BACK, P.OBJ_TRANSIT_Z)
            move_jx(high, f"{label} 바로 위 (1축 회전)")
            move_l(pose, f"{label} 파지점으로 내려가기", vel=P.VEL_SLOW_L)
        else:
            back = P.HANDLE_APPROACH_BACK if back is None else back
            while back > 0:
                try:
                    P.check_pose(offset_tool(pose, -back), label)
                    break
                except ValueError:
                    back -= 10.0
            appr = offset_tool(pose, -max(back, 0.0))
            # 지금 자리에서 바로 직선으로 가면 로봇 몸통 옆을 지나갈 수 있어서(팔 접힘 정지),
            # 홈 → 접근점 '위' → 수직으로 내려가기 → 파지점 순서로
            home()
            high = list(appr); high[2] = max(appr[2] + P.APPROACH_BACK, P.TRANSIT_FREE_Z + 50.0)
            move_jx(high, f"{label} 접근점 위 (1축 회전)")
            move_l(appr, f"{label} 접근", vel=P.VEL_SLOW_L)
            move_l(pose, f"{label} 파지점", vel=P.VEL_SLOW_L)
        gripper.move(grip_w, current=current)     # ★ 조임값: grip_params.py
        if obj:
            held_pub.publish(String(data=obj + "|" + ",".join(f"{v:.2f}" for v in cur())))

    def place(pose, open_w, label):
        confirm(f"{label} 제자리에 내려놓기")
        above = list(pose); above[2] = max(pose[2] + P.RETREAT_UP, P.OBJ_TRANSIT_Z)
        move_l(above, f"{label} 위")
        move_l(pose, f"{label} 내려놓기 (바닥에 맞닿게)", vel=P.VEL_SLOW_L)
        gripper.move(open_w, wait=0.3)            # 느슨하게 풀기
        held_pub.publish(String(data=""))
        above = list(pose); above[2] = max(pose[2] + P.RETREAT_UP, P.OBJ_TRANSIT_Z)
        move_l(above, f"{label}에서 위로 빠지기", vel=P.VEL_SLOW_L)

    # ---------------- 제스처 (인사 · 끄덕 · 도리도리) ----------------
    def bow():
        confirm("손님 인사: ㄱ자 자세")
        home()                       # 항상 홈에서 인사 시작 (낮은 자세에서 바로 숙이면 위험)
        try:                         # 움직이기 전에 인사 자세의 손끝 높이를 계산해서 검사
            from DSR_ROBOT2 import fkin
            tip_z = P.tool_tip(list(fkin(posj(*P.BOW_J))))[2]
        except Exception as e:
            tip_z = None
            log.warning(f"fkin 사용 불가({e}) → 인사 자세 높이 검사 생략")
        if tip_z is not None and tip_z < P.GESTURE_TIP_MIN_Z:
            log.error(f"[인사 생략] 인사 자세 손끝 z={tip_z:.0f} < {P.GESTURE_TIP_MIN_Z:.0f} "
                      "(작업대에 닿음) → positions.py BOW_J를 고치세요")
            return
        movej(posj(*P.BOW_J), vel=P.VEL_J * scale, acc=P.ACC_J * scale)
        wait(1.0)
        home(raise_first=False)

    def wiggle(axis, label):
        """그리퍼를 axis(1=y 좌우, 2=z 위아래)로 흔들기. 항상 홈 자세에서 시작하고,
        손끝이 GESTURE_TIP_MIN_Z 밑으로는 절대 안 내려가요 (예전엔 6cm 내려가서 바닥을 뚫었음)."""
        confirm(label)
        home()
        base = cur()
        if axis == 2:
            room = P.tool_tip(base)[2] - P.GESTURE_TIP_MIN_Z      # 내려갈 수 있는 여유
            steps = (+P.NOD_UP, -max(0.0, min(P.NOD_DOWN, room)))
        else:
            steps = (+P.GESTURE_AMP, -P.GESTURE_AMP)
        for _ in range(P.GESTURE_REPEAT):
            for d in steps:
                q = list(base); q[axis] += d
                move_l(q, label, vel=P.VEL_L)
        move_l(base, "원위치")

    def gesture(kind):
        if kind == "greet":
            bow()
        elif kind == "yes":
            wiggle(2, "주문 인식: 그리퍼 위아래 끄덕")
        elif kind == "no":
            wiggle(1, "못 알아들음: 그리퍼 좌우 도리도리")   # 좌우 = 짧은 방향(Y)

    # ---------------- 반죽 붓기 (주전자) ----------------
    def pour_round(hold_sec, tag, molds):
        """비커 손잡이를 옆에서 잡아 들고 → 각 틀의 노란 칸 위에 주둥이를 대고 25° 기울여 붓기.
        틀마다 손잡이·소스통을 피하는 방향을 자동으로 골라요 (positions.plan_pour)."""
        K = P.KETTLE_PICK
        plans = P.plan_pours(molds)          # 틀마다 그리퍼가 팔의 같은 쪽 → J4 안 감김
        pick(K, G.KETTLE_OPEN_WIDTH, G.KETTLE_GRIP_WIDTH, G.KETTLE_GRIP_CURRENT,
             "비커 손잡이", obj="kettle", side=P.CUP_APPROACH)
        up = cur(); up[2] = P.OBJ_TRANSIT_Z
        move_l(up, "비커 수직으로 들기", vel=P.VEL_SLOW_L)

        for i, S, A in plans:
            F, T, H = P._pour_poses(S, A)
            move_jx(H, f"{tag}: {i+1}번 틀 위로 (그리퍼 방향 {A:.0f}°)")
            move_l(F, f"{tag}: 주둥이를 {i+1}번 노란 칸 위로", vel=P.VEL_SLOW_L)
            confirm(f"{tag}: {P.POUR_TILT_DEG:.0f}도 기울이기 → {hold_sec:.0f}초")
            move_l(T, "주둥이 끝을 고정한 채 기울이기", vel=P.VEL_SLOW_L)
            wait(hold_sec)
            move_l(F, "바닥면 수평 복귀", vel=P.VEL_SLOW_L)
            move_l(H, "들어올리기")

        confirm("비커 제자리로")
        hk = list(K); hk[2] = P.OBJ_TRANSIT_Z
        move_jx(hk, "비커 자리 위로 (1축 회전)")
        move_l(K, "비커 내려놓기 (바닥에 맞닿게)", vel=P.VEL_SLOW_L)
        gripper.move(G.KETTLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data=""))
        move_l(offset_tool(K, -P.CUP_APPROACH), "손잡이에서 옆으로 빠지기", vel=P.VEL_SLOW_L)
        out = offset_tool(K, -P.CUP_APPROACH); out[2] = P.OBJ_TRANSIT_Z
        move_l(out, "위로 빠지기", vel=P.VEL_SLOW_L)

    # ---------------- 속재료 (소스통) ----------------
    def dispense_round(kind, molds):
        """kind 소스통을 옆면에서 잡아 들고, 틀 위에서 입구를 아래로 뒤집어 짜기"""
        kind = kind if kind in P.SAUCE_PICKS else "팥"
        bottle = P.SAUCE_PICKS[kind]
        pick(bottle, G.SAUCE_OPEN_WIDTH, G.SAUCE_GRIP_WIDTH,
             G.SAUCE_GRIP_CURRENT, f"{kind} 소스통 옆면 (들 수 있을 만큼만)", obj=f"sauce:{kind}")
        high_z = max(cur()[2] + P.SAUCE_LIFT, P.OBJ_TRANSIT_Z)
        up = cur(); up[2] = high_z
        move_l(up, "입구가 하늘을 향한 채 들기", vel=P.VEL_SLOW_L)

        half = P.FISH_L / 2 - P.DISPENSE_INSET         # 생선 칸(8cm) 가장자리 1cm 안쪽
        z_fl = P.fill_z() + P.SAUCE_GRIP_TO_MOUTH      # 입구가 틀 위 2cm에 오는 높이
        ys = P.SHORT_SIGN * P.DISPENSE_DIR * half
        first = True
        for i, grip in molds:
            md = P.tool_dir(grip)
            tip = P.tool_tip(grip)
            c = [tip[k] + md[k] * P.plate_k() for k in range(2)]   # 먼 판(마주보는 틀) 가운데
            # 판 위에서는 그리퍼를 판 중앙 방향과 90°로 '비켜' 두어야 팔이 덜 접혀요
            ang = math.degrees(math.atan2(c[1], c[0]))
            a = min((ang + 90.0, ang - 90.0),
                    key=lambda t: abs((t - bottle[3] + 180.0) % 360.0 - 180.0))
            # 첫 틀: 입구가 위인 채로 가서 뒤집기 / 두 번째 틀부터: 이미 뒤집힌 자세(C=180) 그대로 이동
            # (예전엔 두 번째 틀로 갈 때도 C=0을 목표로 줘서 이동 중에 통이 다시 바로 서 버렸어요)
            o = [a, 90.0, 0.0 if first else 180.0]
            u = P.tool_dir([0, 0, 0, *o])
            reach = P.TOOL_LENGTH - P.SAUCE_GRIP_DEPTH   # 플랜지 → 통 중심 (깊게 잡은 만큼 짧음)
            fx, fy = c[0] - u[0] * reach, c[1] - u[1] * reach
            move_jx([fx, fy + ys, high_z, *o], f"{i+1}번 틀 위로 (1축 회전)")
            if first:
                confirm("입구를 틀 쪽(아래)으로 뒤집기")
                go_rel_j6(180.0)
                first = False
            o2 = cur()[3:]
            move_l([fx, fy + ys, z_fl, *o2], f"{i+1}번 틀 위 2cm", vel=P.VEL_SLOW_L)
            confirm(f"{kind} 짜기({i+1}번): 아래 → 위 ({2*half:.0f}mm)")
            gripper.move(G.SAUCE_SQUEEZE_WIDTH, wait=0.2)  # ★ 짜는 양: grip_params.py
            move_l([fx, fy - ys, z_fl, *o2], "짜면서 이동", vel=P.POUR_SPEED)
            gripper.move(G.SAUCE_GRIP_WIDTH, wait=0.2)
            move_l([fx, fy - ys, high_z, *o2], "들어올리기")

        confirm("입구를 다시 위로")
        go_rel_j6(-180.0)
        back = list(bottle); back[2] = high_z
        move_jx(back, "제자리 위로 (1축 회전)")
        place(bottle, G.SAUCE_OPEN_WIDTH, f"{kind} 소스통")

    # ---------------- 뚜껑 닫기·뒤집기·열기 (1·2번 틀 각자의 손잡이를 옆에서 잡기) ----------------
    def lid_arc(mi, t0, t1, label, z0=0.0, z1=0.0):
        n = max(1, int(round(abs(t1 - t0) / P.LID_STEP_DEG)))
        for k in range(1, n + 1):
            t = t0 + (t1 - t0) * k / n
            move_l(P.side_lid_pose(t, z0 + (z1 - z0) * k / n, mi), f"{label} {t:.0f}°", vel=P.VEL_SLOW_L)

    def _side_back(pose, label):
        """옆에서 다가가는 거리: 안전 영역 밖이면 10mm씩 줄임"""
        back = P.HANDLE_APPROACH_BACK
        while back > 10:
            try:
                P.check_pose(offset_tool(pose, -back), label)
                break
            except ValueError:
                back -= 10.0
        return back

    def make_j6_room(sweep, label):
        """앞으로 J6가 sweep도 돌아도 ±J6_LIMIT 안에 있도록, 빈손일 때 미리 360° 감아 두기"""
        j6 = list(get_current_posj())[5]
        lim = P.J6_LIMIT
        for turn in (0.0, 360.0, -360.0):
            a = j6 + turn
            if min(a, a + sweep) >= -lim and max(a, a + sweep) <= lim:
                break
        else:
            raise ValueError(f"[안전 정지] {label}: J6 {j6:.0f}°에서 {sweep:+.0f}°를 돌 여유가 없어요 (한계 ±{lim:.0f}°)")
        if turn:
            confirm(f"{label}: 빈손으로 손목(J6) {j6:.0f}° → {j6 + turn:.0f}° 미리 감기 "
                    f"(이어서 {sweep:+.0f}° 돌아도 한계 ±{lim:.0f}° 안)")
            movej(posj(0, 0, 0, 0, 0, turn), vel=P.VEL_J * scale, acc=P.ACC_J * scale, mod=DR_MV_MOD_REL)

    def side_approach(pose, label, j6_sweep=None):
        appr = offset_tool(pose, -_side_back(pose, label))
        high = list(appr); high[2] = P.OBJ_TRANSIT_Z
        move_jx(high, f"{label} 옆 위")
        if j6_sweep:
            make_j6_room(j6_sweep, label)
        move_jx(appr, f"{label} 옆으로 내려가기")
        move_l(pose, f"{label} 잡는 점", vel=P.VEL_SLOW_L)

    def side_leave(label):
        here = cur()
        out = offset_tool(here, -_side_back(here, label))
        move_l(out, f"{label}에서 옆으로 빠지기", vel=P.VEL_SLOW_L)
        up_ = list(out); up_[2] = P.OBJ_TRANSIT_Z
        move_l(up_, "위로 빠지기", vel=P.VEL_SLOW_L)

    def _R_now():
        return P._zyz_mat(*cur()[3:6])

    def close_lid(mi):
        """가까운 손잡이를 옆에서 잡고 180° 넘겨 먼 판(반죽) 위에 덮기 → 놓고 빠지기"""
        confirm("닫기: 가까운 손잡이를 옆에서 잡기")
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        unwind_if_wound("닫기 전")
        side_approach(P.side_lid_pose(180.0, 0.0, mi), "가까운 손잡이", j6_sweep=P.LID_J6_SWEEP)
        gripper.move(G.HANDLE_GRIP_WIDTH, current=G.HANDLE_GRIP_CURRENT)
        held_pub.publish(String(data="near|" + ",".join(f"{v:.2f}" for v in cur())))
        confirm("경첩을 축으로 180° 넘겨 먼 판 위에 덮기")
        lid_arc(mi, 180.0, 0.0, "닫기", 0.0, P.STACK_T)
        gripper.move(G.STACK_OPEN_WIDTH, wait=0.3)        # 놓되 손은 그 자리 (밑면 익는 동안)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))

    def _end_on_approach(pose, label, open_w):
        gripper.move(open_w, wait=0.3)
        appr = offset_tool(pose, -_side_back(pose, label))
        high = list(appr); high[2] = P.OBJ_TRANSIT_Z
        move_jx(high, f"{label} 위")
        move_jx(appr, f"{label} 앞으로 내려가기")
        move_l(pose, f"{label} 잡는 점", vel=P.VEL_SLOW_L)

    def turn_table(t0, t1, label):
        """경첩 봉 끝을 잡고 돌림판을 t0 → t1 도 수평으로 돌리기"""
        confirm(f"{label}: 경첩 봉 끝 잡기")
        _end_on_approach(P.rod_pose(t0), "경첩 봉 끝", G.HANDLE_OPEN_WIDTH)
        gripper.move(6.0, current=G.HANDLE_GRIP_CURRENT)
        held_pub.publish(String(data="turn|" + ",".join(f"{v:.2f}" for v in cur())))
        confirm(f"{label}: 돌림판 {t1 - t0:+.0f}° 돌리기")
        n = max(1, int(round(abs(t1 - t0) / 15.0)))
        for k in range(1, n + 1):
            move_l(P.rod_pose(t0 + (t1 - t0) * k / n), f"돌리기 {(t1 - t0) * k / n:+.0f}°", vel=P.VEL_SLOW_L)
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))
        side_leave("경첩 봉")

    def flip_stack(mi):
        """닫은 그 자리에서 겹친 손잡이(4cm)를 잡고 들어 180° 뒤집어 받침대 위 제자리에 내려놓기
        (몸통은 그대로, 손잡이만 로봇 쪽) → 잡은 채 앞면 익히기 → (FLIP_BACK) 다시 뒤집어 원래 자리로"""
        R0 = P.stack_rot(mi)
        g = P.pose_with(P.stack_tcp(0.0, mi), R0)
        confirm("뒤집기: 그 자리에서 겹친 손잡이 2장 함께 잡기")
        gripper.move(G.STACK_OPEN_WIDTH, wait=0.3)
        if math.dist(cur()[:3], g[:3]) > 30.0:            # (중간 단계부터 시작해서 손이 딴 데 있을 때)
            side_approach(g, "겹친 손잡이")
        else:
            move_l(g, "겹친 손잡이 가운데로", vel=P.VEL_SLOW_L)
        gripper.move(G.STACK_GRIP_WIDTH, current=G.STACK_GRIP_CURRENT)

        sx, sy = P.flip_shift_vec()

        def lift():
            held_pub.publish(String(data="mold|" + ",".join(f"{v:.2f}" for v in cur())))
            up_ = cur(); up_[2] += P.LIFT_HEIGHT
            move_l(up_, f"+{P.LIFT_HEIGHT/10:.0f}cm 들어올리기", vel=P.VEL_SLOW_L)

        def lower(label):
            dn = cur(); dn[2] -= P.LIFT_HEIGHT
            move_l(dn, f"{label}: -{P.LIFT_HEIGHT/10:.0f}cm 받침대 위에 내려놓기", vel=P.VEL_SLOW_L)

        def slide(k, label):
            q = cur(); q[0] += k * sx; q[1] += k * sy
            move_l(q, f"{label}: 공중에서 수평으로 {abs(P.FLIP_SHIFT)/10:.0f}cm "
                      f"{'로봇 쪽' if k * P.FLIP_SHIFT < 0 else '바깥쪽'}으로", vel=P.VEL_SLOW_L)

        def turn(deg, label):
            confirm(f"{label}: 손목(J6) {deg:+.0f}°")
            return go_rel_j6(deg)

        # 1) 뒤집기: 들기 → 손잡이 축으로 180° (몸통이 바깥쪽 공중으로 넘어감) → 로봇 쪽으로 옮겨
        #    몸통을 받침대 위로 → 내려놓기. 몸통은 제자리, 손잡이만 로봇 쪽으로 바뀜
        j6 = list(get_current_posj())[5]
        lift()
        d = turn(P.FLIP_ANGLE if j6 < 0 else -P.FLIP_ANGLE, "뒤집기")   # J6가 0 쪽으로 풀리는 방향
        slide(+1, "뒤집기")
        lower("뒤집기")
        confirm(f"앞면 익기: 받침대 위에서 잡은 채 {P.BAKE_SEC_SIDE2:.0f}초 대기")
        wait(P.BAKE_SEC_SIDE2)
        # 2) 되돌리기: 순서를 거꾸로 (먼저 바깥쪽으로 옮긴 뒤 돌림 → 돌리는 동안 몸통이 로봇 쪽으로 안 옴)
        if P.FLIP_BACK:
            lift()
            slide(-1, "되돌리기")
            turn(-d, "되돌리기 (뚜껑이 다시 위로)")
            lower("되돌리기 (원래 자리)")
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))

    def open_lid(mi):
        """다 구운 뒤: 겹친 손잡이를 놓고 빠짐 → 위 판 옆면을 옆에서 잡기(아래 손가락은 판 사이 4cm 틈)
        → 경첩을 축으로 펼치기 → 다 펴기 직전에 놓고 빠지기"""
        if not P.FLIP_BACK:
            raise ValueError("[안전 정지] 뒤집은 채로는 열 수 없어요 → positions.py FLIP_BACK = True")
        confirm("열기: 겹친 손잡이 놓고 옆으로 빠지기")
        gripper.move(G.STACK_OPEN_WIDTH, wait=0.3)
        side_leave("겹친 손잡이")
        confirm("열기: 위 판 옆면을 옆에서 잡기 (아래 손가락은 판 사이 틈으로)")
        gripper.move(G.PLATE_OPEN_WIDTH, wait=0.3)
        side_approach(P.plate_open_pose(0.0, mi), "위 판 옆면", j6_sweep=P.OPEN_J6_SWEEP)
        gripper.move(G.PLATE_GRIP_WIDTH, current=G.PLATE_GRIP_CURRENT)
        held_pub.publish(String(data="near|" + ",".join(f"{v:.2f}" for v in cur())))
        end = 180.0 - P.OPEN_RELEASE_DEG
        confirm(f"경첩을 축으로 {end:.0f}° 펼치기")
        n = max(1, int(round(end / P.LID_STEP_DEG)))
        for k in range(1, n + 1):
            t = end * k / n
            move_l(P.plate_open_pose(t, mi), f"열기 {t:.0f}°", vel=P.VEL_SLOW_L)
        gripper.move(G.PLATE_OPEN_WIDTH, wait=0.3)        # 놓으면 남은 각도만큼 받침대 위로 눕혀짐
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))
        side_leave("위 판 옆면")

    # ---------------- 전체 시퀀스 ----------------
    def run_all(fillings, start=0):
        """start = 시작할 단계 번호 (중간부터 다시 할 때)
        0 홈 · 1 1차 반죽 · 2 속재료 · 3 2차 반죽 · 4 뚜껑 닫기 · 5 굽기 대기 · 6 뒤집기 · 7 뚜껑 열기"""
        STEPS = ["홈", "1차 반죽", "속재료", "2차 반죽", "뚜껑 닫기", "굽기 대기", "뒤집기", "뚜껑 열기"]
        n_molds = min(len(fillings) if fillings else 1, len(P.MOLD_GRIPS))
        molds = [(i, P.MOLD_GRIPS[i]) for i in range(max(n_molds, 1))]
        kinds = []
        for i, _ in molds:
            f = fillings[i] if i < len(fillings) and fillings[i] in P.SAUCE_PICKS else "팥"
            kinds.append(f)
        log.info("틀 배정: " + ", ".join(f"{i+1}번={kinds[i]}" for i, _ in molds))
        held_pub.publish(String(data="reset"))          # RViz: 틀을 처음(펼침) 모양으로
        if start:
            log.info(f"▶ {start}단계({STEPS[min(start, len(STEPS) - 1)]})부터 시작")

        def do(n):
            return start <= n

        if do(0):
            confirm("홈 자세")
            home()
            gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.2)
        if do(1):
            pour_round(P.POUR_HOLD1, "1차 반죽", molds)
        if do(2):
            for kind in dict.fromkeys(kinds):          # 주문 순서, 중복 없이
                use = [(i, g) for i, g in molds if kinds[i] == kind]
                dispense_round(kind, use)
        if do(3):
            pour_round(P.POUR_HOLD2, "2차 반죽", molds)
        if do(4):
            for h in range(len(P.HANDLE_SHORTS)):                         # 이번에 쓴 틀만 차례로 닫기
                close_lid(h)
        if do(5):
            confirm(f"밑면 익기: {P.BAKE_SEC_SIDE1:.0f}초 대기 (손은 닫은 손잡이 자리)")
            wait(P.BAKE_SEC_SIDE1)
        if do(6):
            for h in range(len(P.HANDLE_SHORTS)):                         # 틀마다 들어서 180° 뒤집기
                flip_stack(h)
        if do(7):
            for h in range(len(P.HANDLE_SHORTS)):                         # 틀마다 펼치기
                open_lid(h)
        home()
        log.info("✅ 오늘 공정 완료 (틀이 평평하게 열림)")

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        if "--serve" in sys.argv:
            log.info("로봇 연결 확인: 홈 자세로 이동...")
            home(raise_first=False)                 # 여기서 움직이면 드라이버 연결 OK
            log.info("✅ 로봇 연결 확인 완료")
            log.info("대기 중... (start_bake / gesture)")
            while rclpy.ok():
                try:
                    gesture(gestures.get_nowait())
                    continue
                except _queue.Empty:
                    pass
                try:
                    order = orders.get(timeout=0.3)
                except _queue.Empty:
                    continue
                log.info(f"🧾 주문 수신: {order}")
                wait(0.5)                                  # 인사·끄덕 제스처가 먼저 끝나도록
                while True:
                    try:
                        gesture(gestures.get_nowait())
                    except _queue.Empty:
                        break
                body, _, st = order.partition("@")          # "팥,슈크림@2" = 이 손님의 3번째 붕어빵부터
                fillings = [f.strip() for f in body.split(",") if f.strip()]
                try:
                    run_all(fillings or ["팥"], int(st) if st.strip().isdigit() else 0)
                    done_pub.publish(Bool(data=True))
                except ValueError as e:                   # 안전 정지/도달 실패 → 알리고 계속 대기
                    log.error(str(e))
                    held_pub.publish(String(data=""))
                    done_pub.publish(Bool(data=False))
                    try:
                        home()
                    except Exception:
                        pass
                log.info("대기 중...")
        else:
            fillings = [a for a in sys.argv[1:] if not a.startswith("--") and a in P.SAUCE_PICKS]
            run_all(fillings or ["팥"])

    except ValueError as e:
        log.error(str(e))
    except KeyboardInterrupt:
        log.warning("사용자가 중단했어요")
    finally:
        ex.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
