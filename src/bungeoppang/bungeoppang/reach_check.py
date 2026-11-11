#!/usr/bin/env python3
"""배치 닿는지 검사 - 로봇은 움직이지 않음 (IK + /check_state_validity 만 물어봄)

공정에서 쓰는 주요 목표 자세마다 IK 해가 있는지, 그 자세에서 부딪히는 게 없는지 표로 출력.
ROBOT_MODEL 로 고른 layout.py 배치를 MoveIt 장면에 올린 뒤 검사함.
  ROBOT_MODEL=e0509 ros2 run bungeoppang reach_check
주의: 목표 지점(끝점)만 검사. 그 사이 직선 경로의 관절 튐은 pour_demo 실행 때 따로 확인됨.
"""
import math
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .geom import add, up
from .layout import (MODEL, P, SAUCES, SAUCE_YAW, SAUCE_GRASP_YAW, SAUCE_CARRY_Z, SAUCE_GRASP_ROLL, SAUCE_TURN_X, SIDE, MOLD_YS, MOLD_NAMES,
                     MOLD_X0, MOLD_X1, MOLD_XC, MOLD_TOP, MOLD_BASE, MOLD_D, WALL, GAP, TZ, NOZZLE_TIP, KETTLE_GRIP,
                     SPOUT_TIP, FLIP_GRIP_W, FLIP_YAW, FLIP_ROLL0, MACHINE_YAW, mold_w, mw, LID_OPEN, LID_CLOSED, FISH_SIZE, PLATE_XY, PLATE_SIZE,
                     PLATE_SPOTS, PLATE_VIA, TRANSIT_ZS, MACHINE_POS, sauce_grip)
from .pour_demo import Demo


def targets(d):
    """(이름, tcp, pitch, roll, yaw, j4_center) 목록 - pour_demo 공정과 같은 계산"""
    T = []

    def add_t(name, tcp, pitch=0.0, roll=0.0, yaw=0.0, j4=False):
        T.append((name, list(tcp), pitch, roll, yaw, j4))

    # 주전자 (붓기는 주둥이 끝 고정: 기울일수록 손잡이가 들림)
    add_t("주전자 위", up(KETTLE_GRIP, P["approach"]))
    add_t("주전자 잡기", KETTLE_GRIP)
    add_t("주전자 들기", up(KETTLE_GRIP, P["kettle_lift"]))
    dx, dz = SPOUT_TIP
    for i in range(len(MOLD_YS)):
        t = d.pour_tcp(i)
        tip = [t[0] + dx, t[1], t[2] + dz]
        add_t(f"붓기 {MOLD_NAMES[i]}", t)
        for deg in (P["pour_tilt"] / 2, P["pour_tilt"]):
            a = math.radians(deg)
            off = [dx * math.cos(a) + dz * math.sin(a), 0.0, -dx * math.sin(a) + dz * math.cos(a)]
            add_t(f"붓기 {MOLD_NAMES[i]} {deg:.0f}도", add(tip, off, -1.0), deg)
        add_t(f"붓기 {MOLD_NAMES[i]} 위", up(t, P["pour_hop"]))
    add_t("2차 반죽 후 높이", up(d.pour_tcp(len(MOLD_YS) - 1), P["pour2_lift"]))

    # 소스통 (정면으로 옆 잡기 → 높이 들어 손 방향 돌리기 → 뒤집어 짜기)
    z_d = MOLD_TOP + P["nozzle_gap"] + NOZZLE_TIP
    S = [mold_w(i, MOLD_X0 + P["dispense_front"]) for i in range(len(MOLD_YS))]
    E = [mold_w(i, MOLD_X1 - P["dispense_back"]) for i in range(len(MOLD_YS))]
    carry = SAUCE_CARRY_Z
    for kind in SAUCES:
        g = sauce_grip(kind)
        gy = SAUCE_GRASP_YAW[kind]
        R = SAUCE_GRASP_ROLL[kind]
        d.yaw = gy
        pre = add(g, d.dirvec(SIDE), -P["approach"])
        add_t(f"{kind} 접근", pre, SIDE, R, gy)
        add_t(f"{kind} 잡기", g, SIDE, R, gy)
        add_t(f"{kind} 들기", [g[0], g[1], carry], SIDE, R, gy)
        xt = SAUCE_TURN_X
        add_t(f"{kind} 로봇 쪽으로 빠짐", [xt, g[1], carry], SIDE, R, gy)
        add_t(f"{kind} 첫 틀 줄로 운반", [xt, S[0][1], carry], SIDE, R, gy)
        add_t(f"{kind} 손 방향 반쯤", [xt, S[0][1], carry], SIDE, R, (gy + SAUCE_YAW) / 2)
        for i in range(len(MOLD_YS)):
            add_t(f"{kind} {MOLD_NAMES[i]} 줄 방향 돌림", [xt, S[i][1], carry], SIDE, R, SAUCE_YAW)
            add_t(f"{kind} {MOLD_NAMES[i]} 틀 위 운반", [S[i][0], S[i][1], carry], SIDE, R, SAUCE_YAW)
            add_t(f"{kind} 뒤집은 채 {MOLD_NAMES[i]} 위", S[i] + [z_d + P["flip_clear"]], SIDE, R + 180.0, SAUCE_YAW)
            add_t(f"{kind} 뒤집기 전 {MOLD_NAMES[i]}", S[i] + [z_d + P["flip_clear"]], SIDE, R, SAUCE_YAW)
            for nm, pt in (("시작", S[i]), ("끝", E[i])):
                add_t(f"{kind} 짜기 {MOLD_NAMES[i]} {nm}", pt + [z_d], SIDE, R + 180.0, SAUCE_YAW)
    d.yaw = 0.0

    # 철판 뚜껑 손잡이: 열림 → 닫힘 (손 방향 = 기계 방향)
    for k in range(5):
        phi = LID_OPEN + (LID_CLOSED - LID_OPEN) * k / 4
        add_t(f"뚜껑 손잡이 {phi - LID_OPEN:.0f}도", d.handle_w(phi), d.lid_pitch(phi), 0.0, MACHINE_YAW)
    add_t("뚜껑 손잡이 열림 위", up(d.handle_w(LID_OPEN), P["approach"]), d.lid_pitch(LID_OPEN), 0.0, MACHINE_YAW)
    add_t("뚜껑 손잡이 닫힘 위", up(d.handle_w(LID_CLOSED), P["approach"]), d.lid_pitch(LID_CLOSED), 0.0, MACHINE_YAW)

    # 포개진 위·아래 손잡이 한 번에 잡고 철판 통째로 180도 뒤집기 (손잡이 축 = x 방향)
    d.yaw = FLIP_YAW
    pre = add(FLIP_GRIP_W, d.dirvec(SIDE), -P["approach"])
    d.yaw = 0.0
    add_t("뒤집기 손잡이 접근", pre, SIDE, FLIP_ROLL0, FLIP_YAW, True)
    add_t("뒤집기 손잡이 잡기", FLIP_GRIP_W, SIDE, FLIP_ROLL0, FLIP_YAW, True)
    for deg in (90.0, -90.0, 180.0, -180.0):
        add_t(f"뒤집기 손잡이 {deg:+.0f}도", FLIP_GRIP_W, SIDE, FLIP_ROLL0 + deg, FLIP_YAW, True)

    # 붕어빵 꺼내기 → 접시
    carry_f = MOLD_BASE + WALL + GAP + FISH_SIZE[2] / 2 + P["fish_lift"]
    for i in range(len(MOLD_YS)):
        fish = mold_w(i) + [MOLD_BASE + WALL + GAP + FISH_SIZE[2] / 2]
        add_t(f"꺼내기 {MOLD_NAMES[i]}", fish)
        add_t(f"꺼내기 {MOLD_NAMES[i]} 위", up(fish, P["approach"] + MOLD_D))
    if PLATE_VIA:
        add_t("접시 가는 길 거치는 곳", [PLATE_VIA[0], PLATE_VIA[1], carry_f])
    for n, (sx, sy, lv) in enumerate(PLATE_SPOTS):
        spot = [PLATE_XY[0] + sx, PLATE_XY[1] + sy,
                TZ + GAP + PLATE_SIZE[2] + GAP + FISH_SIZE[2] / 2 + lv * (FISH_SIZE[2] + GAP)]
        add_t(f"접시 {n + 1}번", spot)
        add_t(f"접시 {n + 1}번 운반 높이", [spot[0], spot[1], carry_f])

    # 안전 높이 경유점
    for h in TRANSIT_ZS:
        for nm, xy in (("주전자", KETTLE_GRIP), ("틀 가운데", mw([MOLD_XC, MACHINE_POS[1]])), ("접시", PLATE_XY)):
            add_t(f"경유 {h:.2f}m {nm} 위", [xy[0], xy[1], h])
    return T


def main():
    rclpy.init()
    node = Node("bungeoppang_reach_check")
    d = Demo(node)
    ex = MultiThreadedExecutor(2)
    ex.add_node(node)
    Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)
    try:
        d.wait_scene_ready()
        d.clear_scene()
        d.build_scene()
        d.ensure_scene()
        d.set_safety_padding()
        if not d.valid_cli.wait_for_service(timeout_sec=5.0):
            d.warn("/check_state_validity 없음 → 충돌 검사 없이 IK만 확인")
        rows = []
        for name, tcp, pitch, roll, yaw, j4 in targets(d):
            d.yaw = yaw
            q = d.ik_pref(*d.flange(tcp, pitch, roll), j4)
            d.contacts = []
            if q is None:
                res = "❌ IK 없음 (안 닿거나 그 자세에서 충돌)"
            elif not d.state_ok(q):
                res = f"❌ 충돌 {d.contacts}"
            else:
                res = "✅"
            rows.append((name, tcp, res))
        d.yaw = 0.0
        print(f"\n===== 도달 검사: {MODEL} (거리 = base_link에서 tcp까지 수평) =====")
        for name, tcp, res in rows:
            r = math.hypot(tcp[0], tcp[1])
            print(f"{res[:1]} {name:<24} tcp=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) r={r:.2f}m  {res[1:].strip()}")
        bad = [r for r in rows if not r[2].startswith("✅")]
        print(f"===== {len(rows) - len(bad)}/{len(rows)} 통과" + (f", 실패 {len(bad)}개 → layout.py PROFILES['{MODEL}'] 조정" if bad else " 🎉") + " =====")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
