#!/usr/bin/env python3
"""붕어빵 전체 공정 데모 (틀 3개) - RViz2 + MoveIt 2 + pymoveit2, 두산 m1013
1차 반죽(왼→중→오) → 팥소(왼→중→오, 각 아래→위) → 2차 반죽 → 뚜껑 닫기/굽기/열기 → 붕어빵 3개 접시로

이동 원칙
- 물건을 들고 있을 때: 위로 → 수평 → 아래로, 직선 이동만
- 직선 경로는 1cm/2도 간격으로 가장 가까운 관절 해를 이어서 계산, 관절이 튀는 해는 거부 → 360도 회전 없음
- 예외적인 회전: 주전자 40도 기울이기, 소스통 입구를 틀 쪽으로 180도(되돌릴 땐 반대로 180도)
- 그리퍼는 RViz2에 MarkerArray(/bungeoppang/gripper)로 표시
"""
import math
import random
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from pymoveit2 import MoveIt2

# ---------------- 로봇 설정 (두산 m1013) ----------------
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
BASE_LINK = "base_link"
EE_LINK = "link_6"
GROUP = "manipulator"
HOME = [0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0]
WRAP_JOINTS = (0, 3, 5)
YAW_OFFSET = 180.0   # 손 방향을 홈 자세와 맞춤 (joint_6이 0도 근처에 머물게)

TIME_SCALE = 0.1     # 대기 시간 배율: 테스트 0.1 (3분 → 18초), 실제 1.0

# ---------------- 작업 파라미터 (실물 튜닝 대상) ----------------
P = {
    "vel": 0.2,
    "joint_vmax": 2.0,       # 관절 최대 속도 (rad/s) × vel 비율로 사용
    "lin_speed": 0.10,       # 직선 이동 속도 (m/s)
    "ang_speed": 20.0,       # 회전 속도 (도/s)
    "path_step": 0.01,       # 직선 경로를 나누는 간격 (1cm)
    "path_deg": 2.0,         # 회전 경로를 나누는 간격 (2도)
    "max_jump_deg": 25.0,    # 한 간격에서 관절이 이보다 크게 바뀌면 거부
    "tcp": 0.15,             # 플랜지 ~ 그리퍼 손가락 중심 (그리퍼 장착 후 수정)
    "approach": 0.08,        # 잡기 전 떨어져 있는 거리
    # 주전자
    "kettle_lift": 0.20,
    "pour_point": 2 / 3,     # 주둥이가 갈 위치 (틀 아래쪽 끝 기준, 틀 길이 비율)
    "pour_clear": 0.10,      # 주둥이 ~ 틀 윗면 (10cm)
    "pour_tilt": 40.0,
    "pour1_hold": 3.0,
    "pour2_hold": 1.0,
    "pour_hop": 0.08,        # 붓고 세운 뒤 위로 올리는 높이 (8cm)
    "pour2_lift": 0.30,
    # 소스통
    "sauce_lift": 0.30,      # 집은 뒤 들어올리는 높이
    "flip_clear": 0.10,      # 입구를 돌릴 때 틀 위 여유 높이
    "nozzle_gap": 0.005,     # 입구 끝 ~ 틀 윗면 (5mm)
    "dispense_front": 0.04,  # 틀 아래쪽 끝에서 띄우는 거리
    "dispense_back": 0.05,   # 틀 위쪽 끝에서 띄우는 거리
    "dispense_speed": 0.03,  # 짜면서 움직이는 속도 (m/s)
    "dispense_hop": 0.08,    # 틀 사이 옮길 때 올리는 높이
    # 굽기
    "lid_angle": 90.0,
    "lid_step": 5.0,         # 뚜껑을 몇 도씩 움직일지
    "bake_s": 180.0,
    "open_wait": 10.0,
}

# 그리퍼 손가락 간격 (m)
GRIP_OPEN = 0.10
GRIP_KETTLE = 0.02
GRIP_SAUCE = 0.07
GRIP_SQUEEZE = 0.055
GRIP_LID = 0.06
GRIP_FISH = 0.06

# ---------------- 장면 배치 (base_link 기준, m) ----------------
TZ = 0.0
GAP = 0.002
WALL = 0.01
TABLE_SIZE = [1.60, 1.60, 0.04]
TABLE_POS = [0.55, 0.0, TZ - 0.02 - GAP]

# 붕어빵 기계 + 틀 3개 (로봇이 +x를 볼 때 왼쪽 = +y)
MACHINE_SIZE = [0.30, 0.50, 0.10]
MACHINE_POS = [0.80, 0.0, TZ + 0.05]
MOLD_NAMES = ["왼쪽", "중간", "오른쪽"]
MOLD_PITCH = 0.14
MOLD_YS = [MOLD_PITCH, 0.0, -MOLD_PITCH]
MOLD_L, MOLD_W, MOLD_D = 0.20, 0.12, 0.06      # 길이(x, 아래→위), 폭(y), 깊이
MOLD_X0, MOLD_X1 = 0.70, 0.90                  # 아래쪽(앞) 끝, 위쪽(뒤) 끝
MOLD_XC = (MOLD_X0 + MOLD_X1) / 2
MOLD_BASE = TZ + 0.10
MOLD_TOP = MOLD_BASE + MOLD_D
POUR_X = MOLD_X0 + MOLD_L * P["pour_point"]

# 뚜껑: 틀 위쪽(뒤) 끝 경첩, 처음엔 세워진(열린) 상태. 손잡이는 바깥면(닫으면 위를 향함)
HINGE_X, HINGE_Z = MOLD_X1 + 0.005, MOLD_TOP + 0.005
LID_LEN, LID_T = MOLD_L, 0.02
LID_HANDLE = [LID_T + 0.015, LID_LEN - 0.02]   # 경첩 기준 (바깥쪽, 경첩에서 거리) - 열린 상태

# 주전자 (위쪽 손잡이, 주둥이 +x)
KETTLE_R, KETTLE_H = 0.07, 0.16
SPOUT_LEN = 0.14
KX, KY = 0.50, 0.35
KZ = TZ + GAP + KETTLE_H / 2
KETTLE_GRIP = [KX, KY, TZ + GAP + KETTLE_H + 0.05]
SPOUT_TIP = [KETTLE_R + SPOUT_LEN, (KZ + 0.04) - KETTLE_GRIP[2]]

# 소스통 (세워져 있음, 입구 위) - 옆(로봇 기준 오른쪽)에서 몸통 중간을 잡음
SAUCE_XY = [0.60, -0.40]
SAUCE_R, SAUCE_H, NOZZLE_H = 0.035, 0.20, 0.04
SAUCE_GRIP = [SAUCE_XY[0], SAUCE_XY[1], TZ + GAP + SAUCE_H / 2]
NOZZLE_TIP = SAUCE_H / 2 + NOZZLE_H            # 잡는 점 ~ 입구 끝
SIDE = -90.0                                    # 그리퍼를 옆으로 눕힘
SAUCE_YAW = 60.0                                # 비스듬히 잡아 손목이 뚜껑에서 멀어지게

# 접시 / 붕어빵
PLATE_XY, PLATE_R = [0.35, -0.62], 0.15
FISH_SIZE = [0.14, 0.06, 0.025]
PLATE_DY = [0.07, 0.0, -0.07]

KETTLE_PARTS = ["kettle", "kettle_handle", "kettle_post1", "kettle_post2", "kettle_spout"]
SAUCE_PARTS = ["sauce", "sauce_nozzle"]


def gdir(pitch):
    a = math.radians(pitch)
    return [-math.sin(a), 0.0, -math.cos(a)]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def add(v, w, s=1.0):
    return [v[i] + s * w[i] for i in range(3)]


def up(v, dz):
    return [v[0], v[1], v[2] + dz]


def lerp(a, b, r):
    (t0, p0, r0), (t1, p1, r1) = a, b
    return (add(t0, [t1[i] - t0[i] for i in range(3)], r), p0 + (p1 - p0) * r, r0 + (r1 - r0) * r)


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int((t - sec) * 1e9))


class MotionError(RuntimeError):
    pass


class Demo:
    def __init__(self, node):
        self.node = node
        self.step = ""
        self.cur = None        # (tcp, pitch, roll)
        self.yaw = 0.0         # 그리퍼가 향하는 수평 방향 (소스통 잡을 때만 90도)
        self.holding = False
        self.flip_dir = 180.0
        self.finger_w = GRIP_OPEN
        self.m = MoveIt2(
            node=node, joint_names=JOINT_NAMES, base_link_name=BASE_LINK,
            end_effector_name=EE_LINK, group_name=GROUP,
            callback_group=ReentrantCallbackGroup(),
        )
        self.m.max_velocity = P["vel"]
        self.m.max_acceleration = P["vel"]
        self.marker_pub = node.create_publisher(MarkerArray, "/bungeoppang/gripper", 10)
        node.create_timer(0.1, self.publish_gripper)

    # ---------- 로그 / 대기 ----------
    def log(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warn(msg)

    def set_step(self, name):
        self.step = name
        self.log(name)

    def check(self, ok, what):
        if not ok:
            raise MotionError(f"[{self.step}] 실패: {what}")

    def wait(self, sec, label):
        self.log(f"   {label} {sec:.0f}초 대기 (실제 {sec * TIME_SCALE:.1f}초)")
        time.sleep(sec * TIME_SCALE)

    # ---------- 그리퍼 표시 (RViz2 MarkerArray) ----------
    def publish_gripper(self):
        arr = MarkerArray()

        def cube(mid, xyz, size, rgb):
            mk = Marker()
            mk.header.frame_id = EE_LINK
            mk.ns = "gripper"
            mk.id = mid
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = xyz
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = size
            mk.color.r, mk.color.g, mk.color.b = rgb
            mk.color.a = 1.0
            return mk

        t, w = P["tcp"], self.finger_w
        arr.markers.append(cube(0, (0.0, 0.0, (t - 0.03) / 2), (0.06, max(0.14, w + 0.04), t - 0.03), (0.3, 0.3, 0.3)))
        for k, s in enumerate((1, -1)):
            arr.markers.append(cube(1 + k, (0.0, s * (w / 2 + 0.0075), t), (0.03, 0.015, 0.06), (0.85, 0.85, 0.85)))
        self.marker_pub.publish(arr)

    # ---------- 관절 상태 / IK ----------
    def current_joints(self):
        js = self.m.joint_state
        return [js.position[js.name.index(n)] for n in JOINT_NAMES]

    def settle(self, timeout=3.0):
        t0 = time.time()
        prev = self.current_joints()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            now = self.current_joints()
            if max(abs(a - b) for a, b in zip(now, prev)) < 1e-4:
                return
            prev = now
        self.warn(f"[{self.step}] 팔이 {timeout}초 안에 멈추지 않음")

    def jump(self, a, b):
        return max(abs(x - y) for x, y in zip(a, b)) > math.radians(P["max_jump_deg"])

    def center(self, q):
        q = list(q)
        for i in WRAP_JOINTS:
            q[i] = math.atan2(math.sin(q[i]), math.cos(q[i]))
        return q

    def dirvec(self, pitch):
        """그리퍼가 가리키는 방향 (pitch 적용 후 yaw만큼 수평 회전)"""
        d = gdir(pitch)
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        return [c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]]

    def flange(self, tcp, pitch, roll=0.0):
        a = math.radians(pitch)
        r = math.radians(roll + YAW_OFFSET)
        y = math.radians(self.yaw)
        qp = [math.cos(a / 2), 0.0, -math.sin(a / 2), 0.0]
        qy = [0.0, 0.0, math.sin(y / 2), math.cos(y / 2)]
        quat = qmul(qy, qmul(qp, [0.0, 0.0, math.sin(r / 2), math.cos(r / 2)]))
        return add(tcp, self.dirvec(pitch), -P["tcp"]), quat

    def ik_near(self, pos, quat, seed):
        """seed와 가장 가까운 IK 해 (출발점을 조금씩 바꿔 여러 번 계산)"""
        best, best_d = None, 1e9
        seeds = [seed] + [[v + random.uniform(-0.15, 0.15) for v in seed] for _ in range(6)]
        for sd in seeds:
            js = self.m.compute_ik(position=pos, quat_xyzw=quat, start_joint_state=sd)
            if js is None:
                continue
            q = [js.position[js.name.index(n)] for n in JOINT_NAMES]
            for i in WRAP_JOINTS:
                d = q[i] - seed[i]
                q[i] = seed[i] + math.atan2(math.sin(d), math.cos(d))
            dmax = max(abs(a - b) for a, b in zip(q, seed))
            if dmax < best_d:
                best, best_d = q, dmax
            if dmax < math.radians(5):
                break
        return best

    def ik_pref(self, pos, quat):
        """빈손 이동용: 팔꿈치가 홈처럼 위로 꺾이고, 손목 끝이 0도에 가까운 해를 우선"""
        best, best_d = None, 1e9
        d90 = math.pi / 2
        base = [list(HOME),
                [0.0, 0.0, d90, 0.0, -d90, 0.0],
                [0.0, 0.0, d90, d90, d90, 0.0],
                [0.0, 0.0, d90, -d90, d90, 0.0],
                [0.0, 0.0, d90, d90, -d90, 0.0],
                [0.0, 0.0, d90, -d90, -d90, 0.0]]
        seeds = base + [[v + random.uniform(-0.6, 0.6) for v in b] for b in base for _ in range(3)]
        for sd in seeds:
            js = self.m.compute_ik(position=pos, quat_xyzw=quat, start_joint_state=sd)
            if js is None:
                continue
            q = self.center([js.position[js.name.index(n)] for n in JOINT_NAMES])
            d = sum((a - b) ** 2 for a, b in zip(q, HOME))
            if (q[2] > 0) != (HOME[2] > 0):
                d += 20.0
            if abs(q[5]) > math.radians(120):
                d += 5.0
            if d < best_d:
                best, best_d = q, d
        return best

    # ---------- 궤적 실행 ----------
    def run_traj(self, qs, dts, what):
        if len(qs) < 2:
            return
        m = len(dts)
        r = max(1, int(m * 0.2))
        dts = list(dts)
        for i in range(m):
            f = 1.0
            if i < r:
                f = max(f, 2.5 - 1.5 * i / r)
            if i >= m - r:
                f = max(f, 2.5 - 1.5 * (m - 1 - i) / r)
            dts[i] *= f
        for _ in range(2):
            self.settle()
            tr = JointTrajectory()
            tr.joint_names = JOINT_NAMES
            pts = [self.current_joints()] + qs[1:]
            t = 0.0
            for i, q in enumerate(pts):
                if i > 0:
                    t += dts[i - 1]
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q]
                pt.time_from_start = to_duration(t)
                tr.points.append(pt)
            self.m.execute(tr)
            if self.m.wait_until_executed():
                self.settle()
                return
            self.warn(f"[{self.step}] 실행 중단됨 → 1초 후 재시도")
            time.sleep(1.0)
        self.check(False, what)

    def joints(self, q):
        """관절 이동 (movej 방식)"""
        q0 = self.current_joints()
        n = 20
        qs = [[a + (b - a) * (3 * (k / n) ** 2 - 2 * (k / n) ** 3) for a, b in zip(q0, q)]
              for k in range(n + 1)]
        dq = max(abs(b - a) for a, b in zip(q0, q))
        T = max(0.5, 1.5 * dq / (P["joint_vmax"] * P["vel"]))
        self.run_traj(qs, [T / n] * n, "관절 이동")

    # ---------- 직선 이동 ----------
    def straight(self, target, speed=None):
        """현재 자세 → target(tcp, pitch, roll)을 직선(+각도 보간)으로 이동.
        1cm/2도 간격으로 가장 가까운 관절 해를 이어서 계산하고, 관절이 튀는 해는 거부."""
        a = self.cur
        speed = speed or P["lin_speed"]
        dist = math.dist(a[0], target[0])
        dang = max(abs(target[1] - a[1]), abs(target[2] - a[2]))
        n = max(1, math.ceil(dist / P["path_step"]), math.ceil(dang / P["path_deg"]))
        qs, dts = [self.current_joints()], []
        vj = P["joint_vmax"] * P["vel"]

        def push(qn, frac):
            dq = max(abs(y - x) for x, y in zip(qs[-1], qn))
            dts.append(max(dist * frac / speed, dang * frac / P["ang_speed"], dq / vj, 0.02))
            qs.append(qn)

        def fail(k, qa, qb, wp):
            if qb is None:
                why = "IK 해 없음 (팔이 닿지 않거나 충돌로 판단)"
            else:
                j = max(range(6), key=lambda i: abs(qb[i] - qa[i]))
                why = f"joint_{j + 1}이 {math.degrees(abs(qb[j] - qa[j])):.0f}도 튐"
            self.check(False, f"직선 경로 계산 실패 ({k}/{n}) {why}, "
                              f"그 지점 {[round(v, 3) for v in wp[0]]} 각도 {wp[1]:.0f}")

        for k in range(1, n + 1):
            wp = lerp(a, target, k / n)
            qn = self.ik_near(*self.flange(*wp), qs[-1])
            if qn is not None and not self.jump(qs[-1], qn):
                push(qn, 1.0 / n)
                continue
            mid = lerp(a, target, (k - 0.5) / n)
            qm = self.ik_near(*self.flange(*mid), qs[-1])
            if qm is None or self.jump(qs[-1], qm):
                fail(k, qs[-1], qm, mid)
            qn = self.ik_near(*self.flange(*wp), qm)
            if qn is None or self.jump(qm, qn):
                fail(k, qm, qn, wp)
            push(qm, 0.5 / n)
            push(qn, 0.5 / n)
        self.run_traj(qs, dts, "직선 이동")
        self.cur = (list(target[0]), target[1], target[2])

    def lin(self, tcp, speed=None):
        """직선 이동 (자세 유지)"""
        self.straight((list(tcp), self.cur[1], self.cur[2]), speed)

    def hv(self, tcp, travel_z=None):
        """수직·수평 이동만: 위로 → 수평 → 아래로 (자세 유지)"""
        c = list(self.cur[0])
        tz = max(c[2], tcp[2]) if travel_z is None else travel_z
        if abs(c[2] - tz) > 1e-4:
            self.lin([c[0], c[1], tz])
        if math.dist(c[:2], tcp[:2]) > 1e-4:
            self.lin([tcp[0], tcp[1], tz])
        if abs(tz - tcp[2]) > 1e-4:
            self.lin(tcp)

    def tilt(self, tcp, deg):
        """잡은 점(tcp)을 축으로 deg까지 기울이기 (위치 고정, 회전만)"""
        self.straight((list(tcp), deg, self.cur[2]))

    def go(self, tcp, pitch=0.0, roll=0.0):
        """빈손일 때의 이동. 물건을 들고 있으면 직선 이동만 허용."""
        target = (list(tcp), pitch, roll)
        if self.holding:
            self.straight(target)
            return
        pos, quat = self.flange(*target)
        q = self.ik_pref(pos, quat)
        if q is None:
            self.warn(f"[{self.step}] (빈손) IK 해를 못 찾음 → 일반 계획으로 이동")
            self.m.move_to_pose(position=pos, quat_xyzw=quat)
            self.check(self.m.wait_until_executed(), "일반 이동")
            self.settle()
            q = self.center(self.current_joints())
        self.joints(q)
        self.cur = target

    # ---------- 그리퍼 ----------
    def gripper(self, width, label):
        # TODO: 실제 그리퍼 명령으로 교체 (gripper.py 연동)
        self.finger_w = width
        self.log(f"   gripper {label} (간격 {width * 100:.1f}cm)")
        time.sleep(0.5)

    def grab(self, ids, width):
        self.gripper(width, "CLOSE (조이기)")
        for i in ids:
            self.m.attach_collision_object(id=i, link_name=EE_LINK, touch_links=[EE_LINK])
        self.holding = True
        time.sleep(0.3)

    def release(self, ids):
        self.gripper(GRIP_OPEN, "OPEN (풀기)")
        for i in ids:
            self.m.detach_collision_object(id=i)
        self.holding = False
        time.sleep(0.3)

    # ---------- 장면 ----------
    def box(self, id, size, pos, quat=(0.0, 0.0, 0.0, 1.0)):
        self.m.add_collision_box(id=id, size=size, position=pos, quat_xyzw=list(quat))
        time.sleep(0.05)

    def cyl(self, id, h, r, pos):
        self.m.add_collision_cylinder(id=id, height=h, radius=r, position=pos, quat_xyzw=[0, 0, 0, 1])
        time.sleep(0.05)

    def open_box(self, name, xy, size, z0):
        sx, sy, sz = size
        x, y = xy
        self.box(f"{name}_bottom", [sx, sy, WALL], [x, y, z0 + WALL / 2])
        self.box(f"{name}_wx0", [WALL, sy, sz], [x - sx / 2 + WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wx1", [WALL, sy, sz], [x + sx / 2 - WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wy0", [sx, WALL, sz], [x, y - sy / 2 + WALL / 2, z0 + sz / 2])
        self.box(f"{name}_wy1", [sx, WALL, sz], [x, y + sy / 2 - WALL / 2, z0 + sz / 2])

    def hinge_point(self, i, phi, lx, lz):
        """뚜껑 i의 경첩 기준 점(lx: 바깥쪽, lz: 경첩에서 거리)을 phi도 닫았을 때의 위치"""
        a = math.radians(phi)
        return [HINGE_X + lx * math.cos(a) - lz * math.sin(a), MOLD_YS[i],
                HINGE_Z + lx * math.sin(a) + lz * math.cos(a)]

    def handle_point(self, i, phi):
        return self.hinge_point(i, phi, *LID_HANDLE)

    def set_lid(self, i, phi):
        """뚜껑 i를 경첩 기준 phi도(0 열림, 90 닫힘) 자세로 다시 그림"""
        a = math.radians(-phi)
        quat = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
        self.box(f"lid{i}", [LID_T, MOLD_W, LID_LEN], self.hinge_point(i, phi, LID_T / 2, LID_LEN / 2), quat)
        self.box(f"lid{i}_handle", [0.03, 0.06, 0.03], self.handle_point(i, phi), quat)

    def build_scene(self):
        self.box("table", TABLE_SIZE, TABLE_POS)
        self.box("machine", MACHINE_SIZE, MACHINE_POS)
        for i, y in enumerate(MOLD_YS):
            self.open_box(f"mold{i}", [MOLD_XC, y], [MOLD_L, MOLD_W, MOLD_D], MOLD_BASE)
            self.set_lid(i, 0.0)
        # 주전자
        self.cyl("kettle", KETTLE_H, KETTLE_R, [KX, KY, KZ])
        top = TZ + GAP + KETTLE_H
        self.box("kettle_handle", [0.10, 0.02, 0.02], KETTLE_GRIP)
        self.box("kettle_post1", [0.01, 0.02, 0.05], [KX - 0.045, KY, top + 0.025])
        self.box("kettle_post2", [0.01, 0.02, 0.05], [KX + 0.045, KY, top + 0.025])
        self.box("kettle_spout", [SPOUT_LEN, 0.03, 0.03], [KX + KETTLE_R + SPOUT_LEN / 2, KY, KZ + 0.04])
        # 소스통
        self.cyl("sauce", SAUCE_H, SAUCE_R, SAUCE_GRIP)
        self.cyl("sauce_nozzle", NOZZLE_H, 0.008,
                 [SAUCE_XY[0], SAUCE_XY[1], TZ + GAP + SAUCE_H + NOZZLE_H / 2])
        # 접시
        self.cyl("plate", 0.01, PLATE_R, [PLATE_XY[0], PLATE_XY[1], TZ + GAP + 0.005])
        time.sleep(1.0)

    def pour_tcp(self, y):
        return [POUR_X - SPOUT_TIP[0], y, MOLD_TOP + P["pour_clear"] - SPOUT_TIP[1]]

    # ---------- 공정 단위 ----------
    def pick_kettle(self):
        self.go(up(KETTLE_GRIP, P["approach"]))
        self.lin(KETTLE_GRIP)
        self.grab(KETTLE_PARTS, GRIP_KETTLE)
        self.lin(up(KETTLE_GRIP, P["kettle_lift"]))

    def pour_all(self, hold):
        """틀마다: 40도 기울여 붓기 → 수평으로 세우기 → 8cm 위로 → 옆 틀 위로 수평 이동 → 내려오기"""
        t = self.pour_tcp(MOLD_YS[0])
        self.hv(t)
        for k, name in enumerate(MOLD_NAMES):
            self.log(f"   ▶ {name} 틀")
            self.tilt(t, P["pour_tilt"])
            self.wait(hold, "붓기")
            self.tilt(t, 0.0)
            self.lin(up(t, P["pour_hop"]))
            if k == len(MOLD_NAMES) - 1:
                return
            nxt = self.pour_tcp(MOLD_YS[k + 1])
            self.lin(up(nxt, P["pour_hop"]))
            self.lin(nxt)
            t = nxt

    def place_kettle(self, carry_z):
        self.hv(KETTLE_GRIP, travel_z=carry_z)
        self.release(KETTLE_PARTS)
        self.lin(up(KETTLE_GRIP, P["approach"]))

    def flip_sauce(self, down):
        """그리퍼 축을 중심으로 정확히 180도 (되돌릴 땐 반대 방향 180도). 위치는 고정."""
        tcp, pitch, roll = self.cur
        if down:
            self.flip_dir = -180.0 if self.current_joints()[5] > 0 else 180.0
            self.straight((tcp, pitch, roll + self.flip_dir))
        else:
            self.straight((tcp, pitch, roll - self.flip_dir))

    def move_lid(self, i, phi0, phi1):
        """그리퍼는 아래를 향한 채 손잡이를 따라 원호로 이동, 뚜껑은 경첩을 축으로 따라 회전"""
        n = max(1, int(round(abs(phi1 - phi0) / P["lid_step"])))
        for k in range(1, n + 1):
            phi = phi0 + (phi1 - phi0) * k / n
            self.straight((self.handle_point(i, phi), 0.0, 0.0))
            self.set_lid(i, phi)
            if k % 2 == 0 or k == n:
                self.log(f"      뚜껑 각도 {phi:.0f}도")

    # ---------- 전체 공정 ----------
    def run(self):
        self.build_scene()
        self.set_step("0. 홈 자세")
        self.joints(HOME)
        self.cur = None

        # ===== 1차 반죽 =====
        self.set_step("1. 주전자 손잡이 잡고 수직으로 들어올리기")
        self.pick_kettle()
        lift_z = self.cur[0][2]

        self.set_step("2. 1차 반죽: 왼쪽 → 중간 → 오른쪽 (붓기 → 세우기 → 8cm 위 → 옆 틀로)")
        self.pour_all(P["pour1_hold"])

        self.set_step("3. 들어올린 높이로 원래 자리 이동 → 내려놓고 그리퍼 풀기")
        self.place_kettle(max(lift_z, self.cur[0][2]))

        # ===== 팥소 =====
        self.set_step("4. 소스통으로 → 그리퍼를 옆으로 눕혀 몸통 중간에 → 집기")
        self.yaw = SAUCE_YAW
        pre = add(SAUCE_GRIP, self.dirvec(SIDE), -P["approach"])
        self.go(pre, SIDE, 0.0)
        self.lin(SAUCE_GRIP)
        self.grab(SAUCE_PARTS, GRIP_SAUCE)

        self.set_step("5. 입구가 위를 향한 채 들어올려 왼쪽 틀 위로 → 입구를 틀 쪽(아래)으로")
        z_d = MOLD_TOP + P["nozzle_gap"] + NOZZLE_TIP     # 입구 끝이 틀 5mm 위
        x0 = MOLD_X0 + P["dispense_front"]
        x1 = MOLD_X1 - P["dispense_back"]
        self.lin(up(SAUCE_GRIP, P["sauce_lift"]))         # 수직으로 들어올리기
        self.hv([x0, MOLD_YS[0], z_d + P["flip_clear"]])  # 수평 이동 (→ 필요하면 수직)
        self.flip_sauce(down=True)                        # 그리퍼 축 기준 180도 → 입구 아래

        self.set_step("6. 팥소: 왼쪽 → 중간 → 오른쪽 틀 (각 아래 → 위)")
        for k, (name, y) in enumerate(zip(MOLD_NAMES, MOLD_YS)):
            self.log(f"   ▶ {name} 틀: 아래 → 위")
            if k > 0:
                self.lin([x0, y, z_d + P["dispense_hop"]])  # 옆 틀 아래쪽 끝 위로 수평 이동
            self.lin([x0, y, z_d])                          # 입구를 틀 가까이
            self.gripper(GRIP_SQUEEZE, "더 조이기 (소스 짜기 시작)")
            self.lin([x1, y, z_d], speed=P["dispense_speed"])  # 짜면서 아래 → 위
            self.gripper(GRIP_SAUCE, "조임 풀기 (짜기 멈춤)")
            self.lin([x1, y, z_d + P["dispense_hop"]])

        self.set_step("7. 입구를 다시 위로 → 원래 자리 → 수직으로 내려놓기 → 그리퍼 떼기")
        self.flip_sauce(down=False)
        self.hv(SAUCE_GRIP, travel_z=max(SAUCE_GRIP[2] + P["sauce_lift"], self.cur[0][2]))
        self.release(SAUCE_PARTS)
        self.lin(pre)
        self.yaw = 0.0

        # ===== 2차 반죽 =====
        self.set_step("8. 주전자 손잡이 잡고 수직으로 들어올리기")
        self.pick_kettle()

        self.set_step("9. 2차 반죽: 왼쪽 → 중간 → 오른쪽 (붓기 → 세우기 → 8cm 위 → 옆 틀로)")
        self.pour_all(P["pour2_hold"])
        high_z = self.pour_tcp(MOLD_YS[-1])[2] + P["pour2_lift"]

        self.set_step("10. 30cm 위로 → 수평으로 이동 → 원래 자리에 내려놓기")
        self.place_kettle(high_z)

        # ===== 굽기 =====
        A = P["lid_angle"]
        self.set_step(f"11. 뚜껑 닫기 ({A:.0f}도): 왼쪽 → 중간 → 오른쪽")
        for i, name in enumerate(MOLD_NAMES):
            self.log(f"   ▶ {name} 뚜껑")
            g = self.handle_point(i, 0.0)
            self.go(up(g, P["approach"]))
            self.lin(g)
            self.gripper(GRIP_LID, "CLOSE (손잡이 조이기)")
            self.move_lid(i, 0.0, A)
            self.gripper(GRIP_OPEN, "OPEN (풀기)")
            self.lin(up(self.handle_point(i, A), P["approach"]))

        self.set_step("12. 굽기")
        self.wait(P["bake_s"], "굽기")

        self.set_step(f"13. 뚜껑 열기 ({A:.0f}도 위로): 왼쪽 → 중간 → 오른쪽")
        for i, name in enumerate(MOLD_NAMES):
            self.log(f"   ▶ {name} 뚜껑")
            g = self.handle_point(i, A)
            self.go(up(g, P["approach"]))
            self.lin(g)
            self.gripper(GRIP_LID, "CLOSE (손잡이 조이기)")
            self.move_lid(i, A, 0.0)
            self.gripper(GRIP_OPEN, "OPEN (풀기)")
            self.lin(up(self.handle_point(i, 0.0), P["approach"]))
        self.wait(P["open_wait"], "식히기")

        # ===== 꺼내기 =====
        self.set_step("14. 붕어빵 3개 꺼내서 접시 중앙에 놓기")
        for i, name in enumerate(MOLD_NAMES):
            self.log(f"   ▶ {name} 틀 붕어빵")
            fish = [MOLD_XC, MOLD_YS[i], MOLD_BASE + WALL + GAP + FISH_SIZE[2] / 2]
            self.box(f"fish{i}", FISH_SIZE, fish)
            time.sleep(0.3)
            self.go(up(fish, P["approach"] + MOLD_D))
            self.lin(fish)
            self.grab([f"fish{i}"], GRIP_FISH)
            spot = [PLATE_XY[0], PLATE_XY[1] + PLATE_DY[i], TZ + GAP + 0.01 + GAP + FISH_SIZE[2] / 2]
            self.hv(spot, travel_z=fish[2] + 0.15)
            self.release([f"fish{i}"])
            self.lin(up(spot, P["approach"]))

        self.set_step("15. 홈 복귀")
        self.joints(HOME)
        self.log("done - 붕어빵 3개 완성!")


def main():
    rclpy.init()
    node = Node("bungeoppang_pour_demo")
    demo = Demo(node)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)

    try:
        demo.run()
    except MotionError as e:
        node.get_logger().error(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()