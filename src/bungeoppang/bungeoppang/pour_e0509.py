#!/usr/bin/env python3
"""붕어빵 가게 데모 - RViz2 + MoveIt 2 + pymoveit2, 두산 E0509 (작업대 120x50cm)
손님 도착 → 90도 인사 → 주문 듣기(종류·개수) → 알아들으면 3초 끄덕 / 못 알아들으면 도리도리 후 재질문
→ 3개씩 나눠서 굽기 → 90도 인사 → 다음 손님
한 판: 1차 반죽 → 속재료(팥=빨간 통, 슈크림=노란 통, 아래→위) → 2차 반죽 → 뚜껑 닫기/굽기/열기 → 접시로

이동 원칙
- 물건을 들고 있을 때: 위로 → 수평 → 아래로, 직선 이동만
- 직선 경로는 1cm/2도 간격으로 가장 가까운 관절 해를 이어서 계산, 관절이 튀는 해는 거부 → 360도 회전 없음
- 예외적인 회전: 주전자 40도 기울이기, 소스통 입구를 틀 쪽으로 180도(되돌릴 땐 반대로 180도)
"""
import json
import math
import random
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, ColorRGBA
from moveit_msgs.msg import PlanningScene, ObjectColor, LinkPadding
from moveit_msgs.srv import GetStateValidity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from pymoveit2 import MoveIt2
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

# ---------------- 로봇 설정 (두산 E0509, 작업대 120x50cm) ----------------
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
BASE_LINK = "base_link"
EE_LINK = "link_6"
GROUP = "manipulator"
HOME = [0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0]
WRAP_JOINTS = (0, 3, 5)
YAW_OFFSET = 180.0

TIME_SCALE = 0.1           # 대기 시간 배율: 테스트 0.1, 실제 1.0
CUSTOMER_DIR = -90.0       # 손님 방향: 작업대 아래쪽(-y)
ORDER_WAIT = 25.0          # 주문 답을 기다리는 시간 (초)
ORDER_TRIES = 3            # 주문을 물어보는 최대 횟수

# ---------------- 작업 파라미터 (실물 튜닝 대상) ----------------
P = {
    "vel": 0.2,
    "joint_vmax": 2.0,
    "lin_speed": 0.10,
    "ang_speed": 20.0,
    "path_step": 0.01,
    "path_deg": 2.0,
    "max_jump_deg": 25.0,
    "safety_pad": 0.02,      # 안전 거리: 팔(link_3~5)을 이만큼 두껍게 보고 충돌 검사 (m)
    "check_deg": 3.0,        # 경로를 몇 도 간격으로 충돌 검사할지
    "tcp": 0.15,
    "approach": 0.08,
    # 인사 / 반응
    "bow_shoulder": 45.0,
    "bow_elbow": 45.0,
    "bow_hold": 1.5,
    "yes_s": 3.0,
    "yes_deg": 20.0,
    "no_deg": 15.0,
    # 주전자
    "kettle_lift": 0.20,
    "pour_point": 2 / 3,
    "pour_clear": 0.05,      # 주둥이 끝이 틀 위 5cm. 0.10이면 30도 기울일 때 joint_5가 한계(±135도)를 넘음
    "pour_tilt": 30.0,
    "pour1_hold": 3.0,
    "pour2_hold": 1.0,
    "pour_hop": 0.08,
    "pour2_lift": 0.10,      # E0509는 30cm까지 못 올라감
    # 소스통
    "sauce_lift": 0.30,
    "flip_clear": 0.10,
    "nozzle_gap": 0.005,
    "dispense_front": 0.04,
    "dispense_back": 0.05,
    "dispense_speed": 0.03,
    "dispense_hop": 0.08,
    # 굽기
    "lid_angle": 90.0,
    "lid_step": 5.0,
    "flip_step": 10.0,       # 틀을 몇 도씩 돌릴지
    "bake_s": 180.0,
    "bake_front_s": 90.0,    # 뚜껑 닫고 윗면 굽기 (0이면 닫자마자 뒤집기)
    "bake_back_s": 90.0,     # 앞뒤로 뒤집어서 아랫면 굽기
    "open_wait": 10.0,
}

GRIP_OPEN = 0.10
GRIP_KETTLE = 0.02
GRIP_SAUCE = 0.07
GRIP_SQUEEZE = 0.055
GRIP_LID = 0.06
GRIP_FISH = 0.06

# ---------------- 장면 배치 (base_link 기준, m) ----------------
TZ = -0.025               # 로봇 바닥이 작업대보다 2.5cm 높음
GAP = 0.002
WALL = 0.01
TABLE_SIZE = [1.20, 0.50, 0.04]
TABLE_POS = [0.342, -0.095, TZ - 0.02 - GAP]   # 로봇 중심에서 왼쪽 25.8, 위 15.5, 아래 34.5cm

MACHINE_SIZE = [0.30, 0.44, 0.02]
MACHINE_POS = [0.35, -0.10, TZ + 0.01]
MOLD_NAMES = ["왼쪽", "중간", "오른쪽"]
MOLD_PITCH = 0.14
MOLD_YS = [0.04, -0.10, -0.24]
MOLD_L, MOLD_W, MOLD_D = 0.20, 0.12, 0.06
MOLD_X0, MOLD_X1 = 0.25, 0.45
MOLD_XC = (MOLD_X0 + MOLD_X1) / 2
MOLD_BASE = TZ + 0.10
MOLD_TOP = MOLD_BASE + MOLD_D
POUR_X = MOLD_X0 + MOLD_L * P["pour_point"]

HINGE_X, HINGE_Z = MOLD_X1 + 0.005, MOLD_TOP + 0.005
LID_LEN, LID_T = MOLD_L, 0.02
LID_HANDLE = [LID_T + 0.015, LID_LEN - 0.08]   # E0509가 닿도록 6cm 낮춤

# 틀 뒤집기: 닫힌 틀(틀+뚜껑)을 앞쪽 손잡이를 잡고 길이 방향 축으로 180도 회전
AXIS_Z = (MOLD_BASE + HINGE_Z + LID_T) / 2   # 회전축 높이 (닫힌 틀의 가운데)
AXIS_X = (MOLD_X0 + HINGE_X) / 2             # 회전축 앞뒤 위치 (닫힌 틀의 가운데) - 가로(y) 방향 축
CRANK = [AXIS_X, MOLD_YS[0] + MOLD_W / 2 + 0.045, AXIS_Z]  # 기계 위쪽(+y) 옆면 회전 손잡이
CRANK_SIZE = [0.03, 0.03, 0.03]
CRANK_YAW = -90.0                                       # 그리퍼가 기계 쪽(-y)을 향하게
FLIP_ROLL0 = 90.0
FLIP_HX = MOLD_X0 - 0.02                      # 틀마다 앞쪽 뒤집기 손잡이 위치 (회전축 높이)
FLIP_SIZE = [0.04, 0.03, 0.03]
FLIP_HZ = MOLD_BASE + 0.005                   # 손잡이 높이: 틀 앞쪽 아래 (주전자 바닥과 2cm 여유)
GRIP_FLIP = 0.03

KETTLE_R, KETTLE_H = 0.07, 0.16
SPOUT_LEN = 0.14
KX, KY = 0.03, -0.26          # 주둥이(+x)가 오른쪽 틀에 닿지 않는 자리
KZ = TZ + GAP + KETTLE_H / 2
KETTLE_GRIP = [KX, KY, TZ + GAP + KETTLE_H + 0.05]
SPOUT_TIP = [KETTLE_R + SPOUT_LEN, (KZ + 0.04) - KETTLE_GRIP[2]]

# 소스통 2개: 팥 = 빨간 통 (로봇 가까운 쪽), 슈크림 = 노란 통 (먼 쪽)
SAUCES = {"팥": [0.66, 0.10], "슈크림": [0.80, 0.10]}
SAUCE_IDS = {"팥": "sauce_red", "슈크림": "sauce_cream"}
FILL_COLORS = {"팥": (0.55, 0.12, 0.08), "슈크림": (1.0, 0.85, 0.3)}
FILL_LABELS = {"팥": "RED BEAN", "슈크림": "CUSTARD"}
SAUCE_R, SAUCE_H, NOZZLE_H = 0.035, 0.20, 0.04
NOZZLE_TIP = SAUCE_H / 2 + NOZZLE_H
SIDE = -90.0
SAUCE_GRASP_YAW = 0.0     # 소스통 잡을 때 손 방향 (정면)
SAUCE_YAW = -90.0         # 짤 때 손 방향 (옆, 손목이 작업대 위쪽 바깥)
SAUCE_CARRY_Z = TZ + 0.505   # 소스통 들고 이동하는 높이 (열린 뚜껑 위로)

# 접시: 붕어빵 6개 자리 (2줄 × 3칸)
PLATE_XY, PLATE_SIZE = [-0.17, -0.235], [0.17, 0.22, 0.01]   # 로봇 왼쪽 아래 쟁반
FISH_SIZE = [0.14, 0.06, 0.025]
PLATE_SPOTS = [(0.0, dy, lv) for lv in (0, 1) for dy in (0.065, 0.0, -0.065)]   # 3칸 x 2층
PLATE_VIA = [0.12, -0.28]   # 틀 → 쟁반 갈 때 거치는 곳 (로봇 바로 위를 지나지 않게)
MAX_ORDER = len(PLATE_SPOTS)

# 안전 이동
SAFE_Z = 0.55                                  # 바로 가면 위험할 때 올라갈 기본 높이 (m)
TRANSIT_ZS = (0.55, 0.45, 0.38)               # 높은 것부터 시도 (멀리 있는 목표는 너무 높으면 팔이 안 닿음)
PAD_LINKS = ["link_3", "link_4", "link_5"]   # 안전 거리를 적용할 팔 부분

KETTLE_PARTS = ["kettle", "kettle_handle", "kettle_post1", "kettle_post2", "kettle_spout"]


def sauce_grip(kind):
    x, y = SAUCES[kind]
    return [x, y, TZ + GAP + SAUCE_H / 2]


def sauce_parts(kind):
    return [SAUCE_IDS[kind], SAUCE_IDS[kind] + "_nozzle"]


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
        self.cur = None
        self.yaw = 0.0
        self.cur_yaw = 0.0     # 지금 실제 손 방향 (수평 회전)
        self.holding = False
        self.flip_dir = 180.0
        self.finger_w = GRIP_OPEN
        self.reply = None
        self.customer = False
        self.on_plate = []
        self.m = MoveIt2(
            node=node, joint_names=JOINT_NAMES, base_link_name=BASE_LINK,
            end_effector_name=EE_LINK, group_name=GROUP,
            callback_group=ReentrantCallbackGroup(),
        )
        self.m.max_velocity = P["vel"]
        self.m.max_acceleration = P["vel"]
        self.marker_pub = node.create_publisher(MarkerArray, "/bungeoppang/gripper", 10)
        self.step_pub = node.create_publisher(String, "/bungeoppang/step", 10)
        self.scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
        self.scene_cli = node.create_client(GetPlanningScene, "/get_planning_scene",
                                            callback_group=ReentrantCallbackGroup())
        self.added = {}   # 보낸 물체 기록 (빠진 것 다시 보내기용)
        self.valid_cli = node.create_client(GetStateValidity, "/check_state_validity",
                                            callback_group=ReentrantCallbackGroup())
        self.contacts = []
        self.listen_pub = node.create_publisher(String, "/bungeoppang/listen", 10)
        node.create_timer(0.1, self.publish_markers)
        node.create_subscription(String, "/bungeoppang/order", self.on_order, 10)
        node.create_subscription(String, "/bungeoppang/customer", self.on_customer, 10)

    # ---------- 손님 / 주문 ----------
    def on_customer(self, msg):
        if msg.data == "arrived":
            self.customer = True

    def on_order(self, msg):
        try:
            d = json.loads(msg.data)
            if "counts" in d:
                c = d["counts"]
                self.reply = ("ok", {"팥": int(c.get("팥", 0)), "슈크림": int(c.get("슈크림", 0))}, d.get("text", ""))
            else:
                self.reply = ("fail", None, d.get("text", ""))
        except (ValueError, KeyError, TypeError):
            self.warn(f"주문 형식이 잘못됨: {msg.data}")

    def wait_customer(self):
        self.customer = False
        self.set_step("👀 손님을 기다리는 중... (customer 노드)")
        while not self.customer:
            time.sleep(0.2)

    def take_order(self):
        prompt = "어떤 붕어빵을 몇 개 드릴까요? (예: 팥 두 개, 슈크림 한 개)"
        for _ in range(ORDER_TRIES):
            self.reply = None
            self.listen_pub.publish(String(data=prompt))
            self.set_step(f"🤖 {prompt}")
            t0 = time.time()
            while self.reply is None and time.time() - t0 < ORDER_WAIT:
                time.sleep(0.1)
            if self.reply and self.reply[0] == "ok" and sum(self.reply[1].values()) > 0:
                c = self.reply[1]
                fills = (["팥"] * c["팥"] + ["슈크림"] * c["슈크림"])[:MAX_ORDER]
                self.log(f"📝 주문 확인: 팥 {fills.count('팥')}개, 슈크림 {fills.count('슈크림')}개")
                self.nod_yes()
                return fills
            heard = self.reply[2] if self.reply else ""
            self.log(f"❓ 못 알아들었어요{f' (들은 말: {heard})' if heard else ''}")
            self.shake_no()
            prompt = "죄송해요, 다시 말씀해 주세요. 팥 몇 개, 슈크림 몇 개 드릴까요?"
        self.warn("주문을 받지 못해서 다음 손님을 기다립니다")
        return None

    # ---------- 로그 / 대기 ----------
    def log(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warn(msg)

    def set_step(self, name):
        self.step = name
        self.log(name)
        self.step_pub.publish(String(data=name))

    def check(self, ok, what):
        if not ok:
            raise MotionError(f"[{self.step}] 실패: {what}")

    def wait(self, sec, label):
        self.log(f"   {label} {sec:.0f}초 대기 (실제 {sec * TIME_SCALE:.1f}초)")
        time.sleep(sec * TIME_SCALE)

    # ---------- RViz2 표시: 그리퍼 + 소스통 이름표 ----------
    def publish_markers(self):
        arr = MarkerArray()

        def mk(mid, ns, frame, typ, xyz, size, rgb, text=""):
            m = Marker()
            m.header.frame_id = frame
            m.ns = ns
            m.id = mid
            m.type = typ
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 1.0
            m.text = text
            return m

        t, w = P["tcp"], self.finger_w
        arr.markers.append(mk(0, "gripper", EE_LINK, Marker.CUBE, (0.0, 0.0, (t - 0.03) / 2),
                              (0.06, max(0.14, w + 0.04), t - 0.03), (0.3, 0.3, 0.3)))
        for k, s in enumerate((1, -1)):
            arr.markers.append(mk(1 + k, "gripper", EE_LINK, Marker.CUBE, (0.0, s * (w / 2 + 0.0075), t),
                                  (0.03, 0.015, 0.06), (0.85, 0.85, 0.85)))
        for k, kind in enumerate(SAUCES):
            x, y = SAUCES[kind]
            arr.markers.append(mk(10 + k, "sauce_labels", BASE_LINK, Marker.CYLINDER, (x, y, TZ + 0.003),
                                  (0.11, 0.11, 0.004), FILL_COLORS[kind]))
            arr.markers.append(mk(20 + k, "sauce_labels", BASE_LINK, Marker.TEXT_VIEW_FACING,
                                  (x, y, TZ + SAUCE_H + NOZZLE_H + 0.06), (0.0, 0.0, 0.04),
                                  FILL_COLORS[kind], FILL_LABELS[kind]))
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

    def ik_pref(self, pos, quat, j4_center=False):
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
            if j4_center and abs(q[3]) > math.radians(120):
                d += 20.0
            if d < best_d:
                best, best_d = q, d
        return best

    # ---------- 충돌 검사 ----------
    def state_ok(self, q):
        """관절 값 q 자세에서 부딪히는 게 없는지 MoveIt에 물어봄"""
        if not self.valid_cli.service_is_ready():
            return True
        req = GetStateValidity.Request()
        req.group_name = GROUP
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(JOINT_NAMES)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.valid_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 1.0:
            time.sleep(0.002)
        if not fut.done() or fut.result() is None:
            return True
        res = fut.result()
        if not res.valid:
            self.contacts = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts})[:3]
        return res.valid

    def path_clear(self, q0, q1):
        """q0 → q1 관절 경로를 check_deg 간격으로 잘게 나눠 전부 검사"""
        n = max(1, math.ceil(max(abs(b - a) for a, b in zip(q0, q1)) / math.radians(P["check_deg"])))
        for k in range(1, n + 1):
            if not self.state_ok([a + (b - a) * k / n for a, b in zip(q0, q1)]):
                return False
        return True

    def path_list_clear(self, qs):
        return all(self.path_clear(a, b) for a, b in zip(qs, qs[1:]))

    def set_safety_padding(self):
        """팔 일부를 safety_pad만큼 두껍게 보고 충돌 검사 → 물체와 항상 그만큼 떨어져서 움직임"""
        ps = PlanningScene()
        ps.is_diff = True
        for link in PAD_LINKS:
            lp = LinkPadding()
            lp.link_name = link
            lp.padding = float(P["safety_pad"])
            ps.link_padding.append(lp)
        self.scene_pub.publish(ps)
        time.sleep(0.3)
        self.log(f"🛡️ 안전 거리 {P['safety_pad'] * 100:.0f}cm 적용 ({', '.join(PAD_LINKS)})")

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

    def joints(self, q, T=None, checked=False):
        """관절 이동 (movej 방식). 실행 전에 경로 전체를 충돌 검사"""
        q0 = self.current_joints()
        if not checked:
            self.check(self.path_clear(q0, q), f"관절 이동 경로에 충돌 {self.contacts}")
        n = 20
        qs = [[a + (b - a) * (3 * (k / n) ** 2 - 2 * (k / n) ** 3) for a, b in zip(q0, q)]
              for k in range(n + 1)]
        dq = max(abs(b - a) for a, b in zip(q0, q))
        if T is None:
            T = max(0.5, 1.5 * dq / (P["joint_vmax"] * P["vel"]))
        self.run_traj(qs, [T / n] * n, "관절 이동")

    # ---------- 직선 이동 ----------
    def straight(self, target, speed=None):
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
        self.check(self.path_list_clear(qs), f"직선 경로에 충돌 {self.contacts}")
        self.run_traj(qs, dts, "직선 이동")
        self.cur = (list(target[0]), target[1], target[2])
        self.cur_yaw = self.yaw

    def lin(self, tcp, speed=None):
        self.straight((list(tcp), self.cur[1], self.cur[2]), speed)

    def hv(self, tcp, travel_z=None):
        c = list(self.cur[0])
        tz = max(c[2], tcp[2]) if travel_z is None else travel_z
        if abs(c[2] - tz) > 1e-4:
            self.lin([c[0], c[1], tz])
        if math.dist(c[:2], tcp[:2]) > 1e-4:
            self.lin([tcp[0], tcp[1], tz])
        if abs(tz - tcp[2]) > 1e-4:
            self.lin(tcp)

    def turn_yaw(self, yaw1, step=5.0):
        """손 위치는 그대로, 수평 방향(yaw)만 조금씩 돌리기 (물건을 높이 든 채로)"""
        y0 = self.cur_yaw
        n = max(1, math.ceil(abs(yaw1 - y0) / step))
        for k in range(1, n + 1):
            self.yaw = y0 + (yaw1 - y0) * k / n
            self.straight((list(self.cur[0]), self.cur[1], self.cur[2]))
        self.yaw = yaw1

    def tilt(self, tcp, deg):
        self.straight((list(tcp), deg, self.cur[2]))

    def tilt_tip(self, tip, a0, a1, step=5.0):
        """주둥이 끝(tip)을 제자리에 두고 a0 → a1도 기울이기 (손잡이 쪽이 들리며 기울어짐)"""
        dx, dz = SPOUT_TIP
        n = max(1, math.ceil(abs(a1 - a0) / step))
        for k in range(1, n + 1):
            t = math.radians(a0 + (a1 - a0) * k / n)
            off = [dx * math.cos(t) + dz * math.sin(t), 0.0, -dx * math.sin(t) + dz * math.cos(t)]
            self.straight((add(tip, off, -1.0), a0 + (a1 - a0) * k / n, self.cur[2]))

    def go(self, tcp, pitch=0.0, roll=0.0, j4_center=False):
        """빈손 이동: 바로 가는 길에 충돌이 있으면 → 안전 높이로 올라가서 → 목표 위로 → 수직으로 내려감.
        물건을 들고 있으면 직선 이동만 허용."""
        target = (list(tcp), pitch, roll)
        if self.holding:
            self.straight(target)
            return
        q = self.ik_pref(*self.flange(*target), j4_center)
        if q is not None and self.path_clear(self.current_joints(), q):
            self.joints(q, checked=True)
            self.cur = target
            self.cur_yaw = self.yaw
            return
        why = "IK 해 없음" if q is None else f"충돌 {self.contacts}"
        plan = None
        for h0 in TRANSIT_ZS:
            h = max(h0, tcp[2])
            q_up = None
            if self.cur is not None and self.cur[0][2] < h - 1e-3:
                c = self.cur[0]
                keep = self.yaw
                self.yaw = self.cur_yaw
                q_up = self.ik_near(*self.flange([c[0], c[1], h], self.cur[1], self.cur[2]), self.current_joints())
                self.yaw = keep
                if q_up is None:
                    continue                                   # 지금 자리에서 그 높이까지 못 올라감
            above = [tcp[0], tcp[1], h]
            qa = self.ik_pref(*self.flange(above, pitch, roll), j4_center)
            if qa is None:
                continue                                       # 목표 위 그 높이에 팔이 안 닿음
            if not self.path_clear(q_up if q_up is not None else self.current_joints(), qa):
                continue                                       # 그 높이로 가도 부딪힘
            plan = (h, q_up is not None, above, qa)
            break
        self.check(plan is not None, f"올라갔다 가는 길도 못 찾음 (목표 {[round(v, 3) for v in tcp]}, {self.contacts})")
        h, lift, above, qa = plan
        self.warn(f"[{self.step}] 바로 가면 위험 ({why}) → {h:.2f}m 높이로 올라갔다가 이동")
        if lift:
            self.lift_here(h)
        self.joints(qa)
        self.cur = (above, pitch, roll)
        self.cur_yaw = self.yaw
        if above[2] > tcp[2] + 1e-4:
            self.lin(tcp)
        self.cur = target

    def lift_here(self, z):
        """지금 손 방향 그대로 수직으로 z 높이까지 올라가기"""
        c = self.cur[0]
        keep = self.yaw
        self.yaw = self.cur_yaw
        try:
            self.lin([c[0], c[1], z])
        finally:
            self.yaw = keep

    def home(self):
        if self.cur is not None and not self.path_clear(self.current_joints(), HOME):
            c = self.cur[0]
            for h in TRANSIT_ZS:
                if c[2] >= h - 1e-3:
                    break
                keep = self.yaw
                self.yaw = self.cur_yaw
                q_up = self.ik_near(*self.flange([c[0], c[1], h], self.cur[1], self.cur[2]), self.current_joints())
                self.yaw = keep
                if q_up is not None and self.path_clear(q_up, HOME):
                    self.warn(f"[{self.step}] 홈으로 바로 가면 위험 ({self.contacts}) → {h:.2f}m 높이로 먼저 올라감")
                    self.lift_here(h)
                    break
        self.joints(HOME)
        self.cur = None

    # ---------- 인사 / 반응 동작 ----------
    def stand(self):
        return [math.radians(CUSTOMER_DIR), 0.0, 0.0, 0.0, 0.0, 0.0]

    def bow(self, label):
        """손님을 향해 90도 인사 (어깨 45도 + 팔꿈치 45도)"""
        self.set_step(f"🙇 {label}")
        s = self.stand()
        self.joints(s)
        b = list(s)
        b[1] += math.radians(P["bow_shoulder"])
        b[2] += math.radians(P["bow_elbow"])
        self.joints(b, T=2.0)
        time.sleep(P["bow_hold"])
        self.joints(s, T=2.0)
        self.cur = None

    def nod_yes(self):
        self.set_step(f"🙆 알겠습니다! ({P['yes_s']:.0f}초 끄덕끄덕)")
        s = self.stand()
        self.joints(s)
        t0 = time.time()
        while time.time() - t0 < P["yes_s"]:
            d = list(s)
            d[4] += math.radians(P["yes_deg"])
            self.joints(d, T=0.4)
            self.joints(s, T=0.4)
        self.cur = None

    def shake_no(self):
        self.set_step("🙅 다시 말씀해 주세요 (도리도리)")
        s = self.stand()
        self.joints(s)
        for sign in (1, -1, 1, -1):
            d = list(s)
            d[0] += sign * math.radians(P["no_deg"])
            self.joints(d, T=0.4)
        self.joints(s, T=0.4)
        self.cur = None

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
        self.added[id] = ("box", size, pos, quat)
        time.sleep(0.05)

    def cyl(self, id, h, r, pos):
        self.m.add_collision_cylinder(id=id, height=h, radius=r, position=pos, quat_xyzw=[0, 0, 0, 1])
        self.added[id] = ("cyl", h, r, pos)
        time.sleep(0.05)

    def color(self, ids, rgb):
        """RViz2에서 물체 색 바꾸기"""
        ps = PlanningScene()
        ps.is_diff = True
        for i in ids:
            oc = ObjectColor()
            oc.id = i
            oc.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0)
            ps.object_colors.append(oc)
        self.scene_pub.publish(ps)
        time.sleep(0.1)

    def paint_scene(self):
        self.set_safety_padding()
        for kind in SAUCES:
            self.color(sauce_parts(kind), FILL_COLORS[kind])
        self.color(KETTLE_PARTS, (0.75, 0.75, 0.8))
        self.color(["plate"], (0.95, 0.95, 0.95))

    def open_box(self, name, xy, size, z0):
        sx, sy, sz = size
        x, y = xy
        self.box(f"{name}_bottom", [sx, sy, WALL], [x, y, z0 + WALL / 2])
        self.box(f"{name}_wx0", [WALL, sy, sz], [x - sx / 2 + WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wx1", [WALL, sy, sz], [x + sx / 2 - WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wy0", [sx, WALL, sz], [x, y - sy / 2 + WALL / 2, z0 + sz / 2])
        self.box(f"{name}_wy1", [sx, WALL, sz], [x, y + sy / 2 - WALL / 2, z0 + sz / 2])

    def hinge_point(self, i, phi, lx, lz):
        a = math.radians(phi)
        return [HINGE_X + lx * math.cos(a) - lz * math.sin(a), MOLD_YS[i],
                HINGE_Z + lx * math.sin(a) + lz * math.cos(a)]

    def handle_point(self, i, phi):
        return self.hinge_point(i, phi, *LID_HANDLE)

    def set_lid(self, i, phi):
        a = math.radians(-phi)
        quat = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
        self.box(f"lid{i}", [LID_T, MOLD_W, LID_LEN], self.hinge_point(i, phi, LID_T / 2, LID_LEN / 2), quat)
        self.box(f"lid{i}_handle", [0.03, 0.06, 0.03], self.handle_point(i, phi), quat)

    def closed_parts(self, i):
        """닫힌 틀 i를 이루는 물체들 (id, 크기, 중심, 자세) - 뒤집기 전 기준"""
        y = MOLD_YS[i]
        sx, sy, sz = MOLD_L, MOLD_W, MOLD_D
        x, z0 = MOLD_XC, MOLD_BASE
        q0 = (0.0, 0.0, 0.0, 1.0)
        a = math.radians(-90.0)
        ql = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
        return [
            (f"mold{i}_bottom", [sx, sy, WALL], [x, y, z0 + WALL / 2], q0),
            (f"mold{i}_wx0", [WALL, sy, sz], [x - sx / 2 + WALL / 2, y, z0 + sz / 2], q0),
            (f"mold{i}_wx1", [WALL, sy, sz], [x + sx / 2 - WALL / 2, y, z0 + sz / 2], q0),
            (f"mold{i}_wy0", [sx, WALL, sz], [x, y - sy / 2 + WALL / 2, z0 + sz / 2], q0),
            (f"mold{i}_wy1", [sx, WALL, sz], [x, y + sy / 2 - WALL / 2, z0 + sz / 2], q0),
            (f"lid{i}", [LID_T, MOLD_W, LID_LEN], self.hinge_point(i, 90.0, LID_T / 2, LID_LEN / 2), ql),
            (f"lid{i}_handle", [0.03, 0.06, 0.03], self.handle_point(i, 90.0), ql),
        ]

    def set_flip(self, i, theta):
        """닫힌 틀 i를 가로 회전축(y 방향) 기준으로 앞뒤로 theta도 넘긴 모습으로 다시 그림"""
        t = math.radians(theta)
        c, s = math.cos(t), math.sin(t)
        qr = [0.0, math.sin(t / 2), 0.0, math.cos(t / 2)]
        for pid, size, ctr, q in self.closed_parts(i):
            dx, dz = ctr[0] - AXIS_X, ctr[2] - AXIS_Z
            new = [AXIS_X + dx * c + dz * s, ctr[1], AXIS_Z - dx * s + dz * c]
            self.m.add_collision_box(id=pid, size=size, position=new, quat_xyzw=qmul(qr, list(q)))
            time.sleep(0.01)

    def flip_mold(self, i, back):
        """틀 i의 앞쪽 손잡이를 위에서 잡고, 회전축을 중심으로 위로 원을 그리며 뒤쪽까지 넘김 → 앞뒤로 180도.
        back=True면 뒤로 넘어간 손잡이를 잡고 반대로 되돌림 (그리퍼는 계속 아래를 향함)"""
        y = MOLD_YS[i]
        dx, dz = FLIP_HX - AXIS_X, FLIP_HZ - AXIS_Z

        def handle_at(phi):           # phi: 0 = 앞(원래 자리), 180 = 뒤(뒤집힌 자리)
            a = math.radians(phi)
            return [AXIS_X + dx * math.cos(a) + dz * math.sin(a), y,
                    AXIS_Z - dx * math.sin(a) + dz * math.cos(a)]

        p0, p1 = (180.0, 0.0) if back else (0.0, 180.0)
        self.log(f"   ▶ {MOLD_NAMES[i]} 틀 {'윗면으로 되돌리기' if back else '앞뒤로 뒤집기'}")
        g = handle_at(p0)
        self.go(up(g, P["approach"]))
        self.lin(g)
        self.gripper(GRIP_FLIP, "CLOSE (뒤집기 손잡이 조이기)")
        n = max(1, int(round(180.0 / P["flip_step"])))
        for k in range(1, n + 1):
            phi = p0 + (p1 - p0) * k / n
            self.straight((handle_at(phi), 0.0, 0.0))
            self.set_flip(i, phi)
            if k % 6 == 0 or k == n:
                self.log(f"      {abs(phi - p0):.0f}도")
        self.gripper(GRIP_OPEN, "OPEN (풀기)")
        self.lin(up(handle_at(p1), P["approach"]))

    def flip_all(self, back):
        """기계 왼쪽 옆 회전 손잡이를 잡고 손목 끝만 180도 돌림 → 톱니로 연결된 틀 3개가 함께 뒤집힘
        (되돌릴 땐 반대 방향 180도)"""
        self.log(f"   ▶ 회전 손잡이로 틀 3개 앞뒤로 {'되돌리기 (윗면 위로)' if back else '뒤집기 (아랫면 위로)'}")
        self.yaw = CRANK_YAW
        g = list(CRANK)
        pre = add(g, self.dirvec(SIDE), -P["approach"])
        self.go(pre, SIDE, FLIP_ROLL0, j4_center=True)
        self.lin(g)
        self.gripper(GRIP_FLIP, "CLOSE (회전 손잡이 조이기)")
        d = -180.0 if self.current_joints()[5] > 0 else 180.0
        start = 180.0 if back else 0.0
        n = max(1, int(round(180.0 / P["flip_step"])))
        for k in range(1, n + 1):
            self.straight((g, SIDE, FLIP_ROLL0 + d * k / n))
            for i in range(len(MOLD_YS)):
                self.set_flip(i, start - d * k / n)
            if k % 6 == 0 or k == n:
                self.log(f"      {abs(d * k / n):.0f}도")
        self.gripper(GRIP_OPEN, "OPEN (풀기)")
        self.lin(pre)
        self.yaw = 0.0

    def wait_scene_ready(self):
        """MoveIt이 물체를 받을 준비가 될 때까지 대기"""
        self.log("⏳ MoveIt 장면 준비 대기...")
        while not self.scene_cli.wait_for_service(timeout_sec=1.0):
            pass
        time.sleep(3.0)

    def world_ids(self):
        """MoveIt 장면에 실제로 들어 있는 물체 이름 목록"""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = self.scene_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return None
        sc = fut.result().scene
        return {o.id for o in sc.world.collision_objects} | \
               {a.object.id for a in sc.robot_state.attached_collision_objects}

    def ensure_scene(self):
        """보낸 물체가 전부 장면에 들어갔는지 확인하고, 빠진 건 다시 보냄"""
        for _ in range(5):
            ids = self.world_ids()
            if ids is None:
                self.warn("장면 확인 서비스 응답 없음 → 확인 생략")
                return
            missing = [i for i in self.added if i not in ids]
            if not missing:
                self.log(f"✅ 장면 물체 {len(self.added)}개 모두 확인")
                return
            self.warn(f"빠진 물체 {len(missing)}개 다시 보내는 중: {', '.join(missing[:6])}{' ...' if len(missing) > 6 else ''}")
            for i in missing:
                kind, *a = self.added[i]
                if kind == "box":
                    self.box(i, a[0], a[1], a[2])
                else:
                    self.cyl(i, a[0], a[1], a[2])
            time.sleep(1.0)
        self.warn("일부 물체가 계속 빠져 있음 - RViz2에서 확인해주세요")

    def build_scene(self):
        self.box("table", TABLE_SIZE, TABLE_POS)
        self.box("machine", MACHINE_SIZE, MACHINE_POS)
        # 받침 기둥: 바닥판에서 회전축 높이까지, 틀과 틀 사이 틈과 양쪽 바깥 (틀은 기둥 끝에 걸려서 앞뒤로 돎)
        post_h = AXIS_Z - (TZ + MACHINE_SIZE[2])
        gap_ys = [MOLD_YS[0] + MOLD_W / 2 + (MOLD_PITCH - MOLD_W) / 2] + \
                 [(a + b) / 2 for a, b in zip(MOLD_YS, MOLD_YS[1:])] + \
                 [MOLD_YS[-1] - MOLD_W / 2 - (MOLD_PITCH - MOLD_W) / 2]
        for k, py in enumerate(gap_ys):
            self.box(f"mold_post{k}", [0.012, 0.012, post_h], [AXIS_X, py, TZ + MACHINE_SIZE[2] + post_h / 2])
        # 회전 손잡이: 왼쪽 바깥 기둥 → 짧은 축 → 손잡이 (돌리면 연결된 틀 3개가 함께 돎)
        self.box("crank_shaft", [0.012, CRANK[1] - gap_ys[0], 0.012], [AXIS_X, (CRANK[1] + gap_ys[0]) / 2, AXIS_Z])
        self.box("crank", CRANK_SIZE, CRANK)
        for i, y in enumerate(MOLD_YS):
            self.open_box(f"mold{i}", [MOLD_XC, y], [MOLD_L, MOLD_W, MOLD_D], MOLD_BASE)
            self.set_lid(i, 0.0)
        self.cyl("kettle", KETTLE_H, KETTLE_R, [KX, KY, KZ])
        top = TZ + GAP + KETTLE_H
        self.box("kettle_handle", [0.10, 0.02, 0.02], KETTLE_GRIP)
        self.box("kettle_post1", [0.01, 0.02, 0.05], [KX - 0.045, KY, top + 0.025])
        self.box("kettle_post2", [0.01, 0.02, 0.05], [KX + 0.045, KY, top + 0.025])
        self.box("kettle_spout", [SPOUT_LEN, 0.03, 0.03], [KX + KETTLE_R + SPOUT_LEN / 2, KY, KZ + 0.04])
        for kind in SAUCES:
            g = sauce_grip(kind)
            sid, nid = sauce_parts(kind)
            self.cyl(sid, SAUCE_H, SAUCE_R, g)
            self.cyl(nid, NOZZLE_H, 0.008, [g[0], g[1], TZ + GAP + SAUCE_H + NOZZLE_H / 2])
        self.box("plate", PLATE_SIZE, [PLATE_XY[0], PLATE_XY[1], TZ + GAP + 0.005])
        self.box("robot_plate", [0.21, 0.17, -GAP - TZ], [0.0, 0.0, (TZ - GAP) / 2])   # 로봇 받침 2.5cm
        time.sleep(1.0)

    def clear_plate(self):
        for fid in self.on_plate:
            self.m.remove_collision_object(id=fid)
            self.added.pop(fid, None)
            time.sleep(0.05)
        self.on_plate = []

    def pour_tcp(self, y):
        return [POUR_X - SPOUT_TIP[0], y, MOLD_TOP + P["pour_clear"] - SPOUT_TIP[1]]

    # ---------- 공정 단위 ----------
    def pick_kettle(self):
        self.go(up(KETTLE_GRIP, P["approach"]))
        self.lin(KETTLE_GRIP)
        self.grab(KETTLE_PARTS, GRIP_KETTLE)
        self.lin(up(KETTLE_GRIP, P["kettle_lift"]))

    def pour_all(self, hold, molds):
        t = self.pour_tcp(MOLD_YS[molds[0]])
        self.hv(t)
        for k, i in enumerate(molds):
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀")
            tip = [t[0] + SPOUT_TIP[0], t[1], t[2] + SPOUT_TIP[1]]   # 주둥이 끝 = 틀 2/3 지점 위 10cm
            self.tilt_tip(tip, 0.0, P["pour_tilt"])
            self.wait(hold, "붓기")
            self.tilt_tip(tip, P["pour_tilt"], 0.0)
            self.lin(up(t, P["pour_hop"]))
            if k == len(molds) - 1:
                return
            nxt = self.pour_tcp(MOLD_YS[molds[k + 1]])
            self.lin(up(nxt, P["pour_hop"]))
            self.lin(nxt)
            t = nxt

    def place_kettle(self, carry_z):
        self.hv(KETTLE_GRIP, travel_z=carry_z)
        self.release(KETTLE_PARTS)
        self.lin(up(KETTLE_GRIP, P["approach"]))

    def flip_sauce(self, down):
        tcp, pitch, roll = self.cur
        if down:
            self.flip_dir = -180.0 if self.current_joints()[5] > 0 else 180.0
            self.straight((tcp, pitch, roll + self.flip_dir))
        else:
            self.straight((tcp, pitch, roll - self.flip_dir))

    def dispense(self, kind, molds):
        grip = sauce_grip(kind)
        parts = sauce_parts(kind)
        names = ", ".join(MOLD_NAMES[i] for i in molds)
        color = "빨간" if kind == "팥" else "노란"

        self.set_step(f"   [{kind}] {color} 소스통 집기 (옆으로 눕혀 몸통 중간)")
        self.yaw = SAUCE_GRASP_YAW
        pre = add(grip, self.dirvec(SIDE), -P["approach"])
        self.go(pre, SIDE, 0.0)
        self.lin(grip)
        self.grab(parts, GRIP_SAUCE)

        self.set_step(f"   [{kind}] 입구 위로 들어올려 {MOLD_NAMES[molds[0]]} 틀 위로 → 입구를 틀 쪽으로")
        z_d = MOLD_TOP + P["nozzle_gap"] + NOZZLE_TIP
        x0 = MOLD_X0 + P["dispense_front"]
        x1 = MOLD_X1 - P["dispense_back"]
        self.lin([grip[0], grip[1], SAUCE_CARRY_Z])                  # 열린 뚜껑보다 높이
        self.turn_yaw(SAUCE_YAW)                                         # 높이 든 채로 손을 옆으로
        self.hv([x0, MOLD_YS[molds[0]], z_d + P["flip_clear"]], travel_z=SAUCE_CARRY_Z)
        self.flip_sauce(down=True)

        self.set_step(f"   [{kind}] 속재료 넣기: {names} (각 아래 → 위)")
        for k, i in enumerate(molds):
            y = MOLD_YS[i]
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀: 아래 → 위")
            if k > 0:
                self.lin([x0, y, z_d + P["dispense_hop"]])
            self.lin([x0, y, z_d])
            self.gripper(GRIP_SQUEEZE, "더 조이기 (짜기 시작)")
            self.lin([x1, y, z_d], speed=P["dispense_speed"])
            self.gripper(GRIP_SAUCE, "조임 풀기 (짜기 멈춤)")
            self.lin([x1, y, z_d + P["dispense_hop"]])

        self.set_step(f"   [{kind}] 입구를 다시 위로 → 제자리 → 내려놓기")
        c = self.cur[0]
        self.lin([c[0], c[1], z_d + P["flip_clear"]])    # 수직으로 올라가서
        self.lin([x0, c[1], z_d + P["flip_clear"]])      # 뚜껑에서 먼 앞쪽으로 수평 이동
        self.flip_sauce(down=False)                       # 거기서 입구를 위로
        self.lin([x0, self.cur[0][1], SAUCE_CARRY_Z])
        self.lin([grip[0], grip[1], SAUCE_CARRY_Z])
        self.turn_yaw(SAUCE_GRASP_YAW)                                   # 다시 정면으로
        self.lin(grip)
        self.release(parts)
        self.color(parts, FILL_COLORS[kind])
        self.lin(pre)
        self.yaw = 0.0

    def move_lid(self, i, phi0, phi1):
        n = max(1, int(round(abs(phi1 - phi0) / P["lid_step"])))
        for k in range(1, n + 1):
            phi = phi0 + (phi1 - phi0) * k / n
            self.straight((self.handle_point(i, phi), 0.0, 0.0))
            self.set_lid(i, phi)
            if k % 6 == 0 or k == n:
                self.log(f"      뚜껑 각도 {phi:.0f}도")

    # ---------- 한 판 굽기 (틀 최대 3개) ----------
    def bake_batch(self, fillings, tag):
        molds = list(range(len(fillings)))
        A = P["lid_angle"]
        self.home()

        self.set_step(f"{tag} 1. 주전자 잡고 들어올리기")
        self.pick_kettle()
        lift_z = self.cur[0][2]
        self.set_step(f"{tag} 2. 1차 반죽")
        self.pour_all(P["pour1_hold"], molds)
        self.set_step(f"{tag} 3. 주전자 제자리")
        self.place_kettle(max(lift_z, self.cur[0][2]))

        self.set_step(f"{tag} 4. 속재료: " + ", ".join(f"{MOLD_NAMES[i]} {f}" for i, f in enumerate(fillings)))
        for kind in SAUCES:
            use = [i for i, f in enumerate(fillings) if f == kind]
            if use:
                self.dispense(kind, use)

        self.set_step(f"{tag} 5. 주전자 잡고 들어올리기")
        self.pick_kettle()
        self.set_step(f"{tag} 6. 2차 반죽")
        self.pour_all(P["pour2_hold"], molds)
        high_z = self.pour_tcp(MOLD_YS[molds[-1]])[2] + P["pour2_lift"]
        self.set_step(f"{tag} 7. 30cm 위로 → 주전자 제자리")
        self.place_kettle(high_z)

        self.set_step(f"{tag} 8. 뚜껑 닫기")
        for i in range(len(MOLD_YS)):          # 빈 틀 뚜껑도 닫고 엶
            self.log(f"   ▶ {MOLD_NAMES[i]} 뚜껑")
            g = self.handle_point(i, 0.0)
            self.go(up(g, P["approach"]))
            self.lin(g)
            self.gripper(GRIP_LID, "CLOSE (손잡이 조이기)")
            self.move_lid(i, 0.0, A)
            self.gripper(GRIP_OPEN, "OPEN (풀기)")
            self.lin(up(self.handle_point(i, A), P["approach"]))

        if P["bake_front_s"] > 0:
            self.set_step(f"{tag} 9. 윗면 굽기")
            self.wait(P["bake_front_s"], "윗면 굽기")
        self.set_step(f"{tag} 9-1. 회전 손잡이로 틀 3개 한 번에 앞뒤로 뒤집기 (아랫면 굽기)")
        self.home()
        self.flip_all(back=False)
        self.home()
        self.set_step(f"{tag} 9-2. 아랫면 굽기")
        self.wait(P["bake_back_s"], "아랫면 굽기")
        self.set_step(f"{tag} 9-3. 회전 손잡이로 틀 3개 한 번에 윗면으로 되돌리기")
        self.flip_all(back=True)
        self.home()

        self.set_step(f"{tag} 10. 뚜껑 열기")
        for i in range(len(MOLD_YS)):          # 빈 틀 뚜껑도 닫고 엶
            self.log(f"   ▶ {MOLD_NAMES[i]} 뚜껑")
            g = self.handle_point(i, A)
            self.go(up(g, P["approach"]))
            self.lin(g)
            self.gripper(GRIP_LID, "CLOSE (손잡이 조이기)")
            self.move_lid(i, A, 0.0)
            self.gripper(GRIP_OPEN, "OPEN (풀기)")
            self.lin(up(self.handle_point(i, 0.0), P["approach"]))
        self.wait(P["open_wait"], "식히기")

        self.set_step(f"{tag} 11. 붕어빵 꺼내서 접시에 놓기")
        for i in molds:
            n = len(self.on_plate)
            fid = f"fish_{n}"
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀 ({fillings[i]}) → 접시 {n + 1}번 자리")
            fish = [MOLD_XC, MOLD_YS[i], MOLD_BASE + WALL + GAP + FISH_SIZE[2] / 2]
            self.box(fid, FISH_SIZE, fish)
            self.color([fid], FILL_COLORS[fillings[i]])
            time.sleep(0.3)
            self.go(up(fish, P["approach"] + MOLD_D))
            self.lin(fish)
            self.grab([fid], GRIP_FISH)
            dx, dy, lv = PLATE_SPOTS[n]
            spot = [PLATE_XY[0] + dx, PLATE_XY[1] + dy,
                    TZ + GAP + PLATE_SIZE[2] + GAP + FISH_SIZE[2] / 2 + lv * (FISH_SIZE[2] + GAP)]
            tz = fish[2] + 0.30
            self.hv([PLATE_VIA[0], PLATE_VIA[1], tz])
            self.hv(spot, travel_z=tz)
            self.release([fid])
            self.color([fid], FILL_COLORS[fillings[i]])
            self.on_plate.append(fid)
            self.lin(up(spot, P["approach"]))
        self.home()

    def serve(self, fills):
        batches = [fills[i:i + 3] for i in range(0, len(fills), 3)]
        for b, chunk in enumerate(batches, 1):
            self.set_step(f"🔥 {b}/{len(batches)}판 굽기 시작: " + ", ".join(chunk))
            self.bake_batch(chunk, f"[{b}판]")
        self.set_step(f"🐟 붕어빵 {len(fills)}개 완성! (팥 {fills.count('팥')}, 슈크림 {fills.count('슈크림')})")

    def shop(self):
        """가게 운영: 손님 대기 → 90도 인사 → 주문(끄덕/도리도리) → 굽기 → 90도 인사 → 반복"""
        self.log("🤖 pour_e0509.py: E0509 + 작업대 120x50cm 배치")
        self.wait_scene_ready()
        self.build_scene()
        self.ensure_scene()
        self.paint_scene()
        self.home()
        while rclpy.ok():
            self.clear_plate()
            self.wait_customer()
            self.bow("어서오세요! (90도 인사)")
            fills = self.take_order()
            if fills:
                self.serve(fills)
                self.bow("감사합니다, 맛있게 드세요! (90도 인사)")
            self.home()


def main():
    rclpy.init()
    node = Node("bungeoppang_e0509")
    demo = Demo(node)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)

    try:
        demo.shop()
    except MotionError as e:
        node.get_logger().error(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
