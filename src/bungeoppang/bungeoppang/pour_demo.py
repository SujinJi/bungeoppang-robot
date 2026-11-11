#!/usr/bin/env python3
"""붕어빵 가게 데모 - RViz2 + MoveIt 2 + pymoveit2, 두산 로봇 (모델은 ROBOT_MODEL, 배치는 layout.py)
손님 도착 → 90도 인사 → 주문 듣기(종류·개수) → 알아들으면 3초 끄덕 / 못 알아들으면 도리도리 후 재질문
→ 틀 개수(MOLD_COUNT)씩 나눠서 굽기 → 90도 인사 → 다음 손님
한 판: 1차 반죽 → 속재료(팥=빨간 통, 슈크림=노란 통, 아래→위) → 2차 반죽
      → 철판 뚜껑 한 번에 닫기(180도) → 위·아래 손잡이 한 번에 잡고 180도 뒤집기 → 굽기 → 되돌리기 → 열기 → 접시로

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
from std_msgs.msg import String, ColorRGBA
from moveit_msgs.msg import (PlanningScene, ObjectColor, LinkPadding, PlanningSceneComponents,
                             AttachedCollisionObject, CollisionObject)
from moveit_msgs.srv import GetStateValidity, GetPlanningScene, GetPositionIK, ApplyPlanningScene
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from pymoveit2 import MoveIt2

from .geom import gdir, qmul, add, up, lerp, to_duration
from .layout import *  # noqa: F401,F403  로봇 설정 · 파라미터 · 배치 상수 (모델별)
from .layout import MODEL


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
        self.ik_cli = node.create_client(GetPositionIK, "/compute_ik", callback_group=ReentrantCallbackGroup())
        self.apply_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                            callback_group=ReentrantCallbackGroup())
        self.contacts = []
        self.listen_pub = node.create_publisher(String, "/bungeoppang/listen", 10)
        node.create_timer(0.1, self.publish_markers)
        node.create_subscription(String, "/bungeoppang/order", self.on_order, 10)
        node.create_subscription(String, "/bungeoppang/customer", self.on_customer, 10)

    # ---------- 손님 / 주문 ----------
    # camera detection -> if the customer encounters within 100cm, then the robot will bow toward the customer
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
                                  (x, y, SAUCE_Z0 + SAUCE_H + NOZZLE_H + 0.06), (0.0, 0.0, 0.04),
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

    def compute_ik(self, pos, quat, seed):
        """MoveIt IK (충돌 검사 포함). pymoveit2는 시간 제한을 못 줘서 직접 요청함
        (E0509 기본 15ms로는 멀리 뻗는 자세를 못 찾음 → P["ik_timeout"])"""
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GROUP
        ik.ik_link_name = EE_LINK
        ik.avoid_collisions = True
        ik.robot_state.joint_state = JointState(name=list(JOINT_NAMES), position=[float(v) for v in seed])
        ik.pose_stamped.header.frame_id = BASE_LINK
        p = ik.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in quat)
        ik.timeout = to_duration(P["ik_timeout"])
        fut = self.ik_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 2.0:
            time.sleep(0.002)
        if not fut.done() or fut.result() is None or fut.result().error_code.val != 1:
            return None
        return fut.result().solution.joint_state

    def ik_near(self, pos, quat, seed):
        best, best_d = None, 1e9
        seeds = [seed] + [[v + random.uniform(-0.2, 0.2) for v in seed] for _ in range(6)]
        for sd in seeds:
            js = self.compute_ik(pos, quat, sd)
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
        j1 = math.atan2(pos[1], pos[0])           # 목표 쪽으로 돌린 시작점 (멀리 뻗는 자세용)
        base += [[j1, 0.0, d90, 0.0, d90, 0.0], [j1, 0.3, 1.2, 0.0, 1.6, 0.0], [j1, -0.5, 2.0, 0.0, 1.2, 0.0]]
        seeds = base + [[v + random.uniform(-0.6, 0.6) for v in b] for b in base for _ in range(3)]
        def score(q):
            d = sum((a - b) ** 2 for a, b in zip(q, HOME))
            if (q[2] > 0) != (HOME[2] > 0):
                d += 50.0
            if abs(q[5]) > math.radians(120):
                d += 5.0
            if j4_center and abs(q[3]) > math.radians(120):
                d += 50.0
            return d

        for sd in seeds:
            js = self.compute_ik(pos, quat, sd)
            if js is None:
                continue
            q = self.center([js.position[js.name.index(n)] for n in JOINT_NAMES])
            # 같은 손 자세를 내는 반대 손목 해 (joint_4 +180, joint_5 부호 반대, joint_6 +180)
            # KDL이 시작점에 따라 손목 뒤집힌 해만 주는 경우가 있음 → joint_6이 ±180 경계에 붙어 버림
            qf = self.center([q[0], q[1], q[2], q[3] + math.pi, -q[4], q[5] + math.pi])
            for cand, need_check in ((q, False), (qf, True)):
                d = score(cand)
                if d < best_d and (not need_check or self.state_ok(cand)):
                    best, best_d = cand, d
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

    def tilt(self, tcp, deg):
        self.straight((list(tcp), deg, self.cur[2]))

    def turn_yaw(self, yaw1, step=5.0):
        """손 위치는 그대로, 수평 방향(yaw)만 조금씩 돌리기 (물건을 높이 든 채로)"""
        y0 = self.cur_yaw
        n = max(1, math.ceil(abs(yaw1 - y0) / step))
        for k in range(1, n + 1):
            self.yaw = y0 + (yaw1 - y0) * k / n
            self.straight((list(self.cur[0]), self.cur[1], self.cur[2]))
        self.yaw = yaw1

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

    def wait_attached(self, ids, attached, timeout=5.0):
        """ids가 전부 그리퍼에 붙었는지(attached=True) / 전부 떨어졌는지 장면에서 확인. 안 된 건 다시 보냄
        (물체가 많으면 붙이기·떼기 메시지가 늦게 처리돼서 일부가 장면에 남을 수 있음)"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            _, att = self.scene_ids()
            left = [i for i in ids if (i in att) != attached]
            if not left:
                return
            for i in left:
                if attached:
                    self.m.attach_collision_object(id=i, link_name=EE_LINK, touch_links=[EE_LINK])
                else:
                    self.m.detach_collision_object(id=i)
            time.sleep(0.3)
        self.check(False, f"{'붙이기' if attached else '떼기'} 안 끝난 물체: {', '.join(left)}")

    # ---------- 장면 ----------
    def box(self, id, size, pos, quat=(0.0, 0.0, 0.0, 1.0)):
        self.m.add_collision_box(id=id, size=size, position=pos, quat_xyzw=list(quat))
        self.added[id] = ("box", size, pos, quat)

    def mbox(self, id, size, pos, quat=(0.0, 0.0, 0.0, 1.0), wait=0.05):
        """기계 좌표로 적은 상자를 실제 좌표로 바꿔서 장면에 넣음 (기계 방향 MACHINE_YAW)"""
        pw, qw = mw(pos), qmul(MACHINE_Q, list(quat))
        self.m.add_collision_box(id=id, size=size, position=pw, quat_xyzw=qw)
        self.added[id] = ("box", size, pw, qw)
        time.sleep(wait)
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
        self.color(["steel_bottom", "steel_top", "lid_handle", "base_handle"], (0.45, 0.45, 0.5))

    def open_box(self, name, xy, size, z0):
        sx, sy, sz = size
        x, y = xy
        self.box(f"{name}_bottom", [sx, sy, WALL], [x, y, z0 + WALL / 2])
        self.box(f"{name}_wx0", [WALL, sy, sz], [x - sx / 2 + WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wx1", [WALL, sy, sz], [x + sx / 2 - WALL / 2, y, z0 + sz / 2])
        self.box(f"{name}_wy0", [sx, WALL, sz], [x, y - sy / 2 + WALL / 2, z0 + sz / 2])
        self.box(f"{name}_wy1", [sx, WALL, sz], [x, y + sy / 2 - WALL / 2, z0 + sz / 2])

    def hinge_point(self, phi, lx, lz, y):
        """뚜껑 좌표 (lx, lz)를 경첩 기준 phi도 돌린 위치"""
        a = math.radians(phi)
        return [HINGE_X + lx * math.cos(a) - lz * math.sin(a), y,
                HINGE_Z + lx * math.sin(a) + lz * math.cos(a)]

    def handle_point(self, phi):
        """위 철판 손잡이 위치 (기계 좌표, 철판 하나라 틀 개수와 상관없이 하나)"""
        return self.hinge_point(phi, LID_HANDLE[0], LID_HANDLE[1], STEEL_YC)

    def handle_w(self, phi):
        """위 철판 손잡이 위치 (실제 좌표) - 그리퍼가 가는 곳"""
        return mw(self.handle_point(phi))

    def lid_parts(self, phi):
        """뚜껑 쪽 물체들 (틀마다 뚜껑 + 위 철판 + 손잡이)을 phi도로 놓은 모습: (id, 크기, 중심, 자세)"""
        a = math.radians(-phi)
        q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
        parts = [(f"lid{i}", [LID_T, MOLD_W, LID_LEN], self.hinge_point(phi, LID_T / 2, lid_lz(MOLD_XC), y), q)
                 for i, y in enumerate(MOLD_YS)]
        parts.append(("steel_top", [STEEL_T, STEEL_W, STEEL_L],             # 뚜껑 바깥면 위 철판
                      self.hinge_point(phi, LID_T + STEEL_T / 2, lid_lz(STEEL_XC), STEEL_YC), q))
        parts.append(("lid_handle", LID_HANDLE_SIZE, self.handle_point(phi), q))
        return parts

    def base_parts(self):
        """아래 쪽 물체들 (틀마다 바닥·벽 + 아래 철판), 뒤집기 전 자리"""
        q0 = (0.0, 0.0, 0.0, 1.0)
        sx, sy, sz = MOLD_L, MOLD_W, MOLD_D
        x, z0 = MOLD_XC, MOLD_BASE
        parts = [("steel_bottom", [STEEL_L, STEEL_W, STEEL_T], [STEEL_XC, STEEL_YC, z0 - STEEL_T / 2], q0),
                 ("base_handle", BASE_HANDLE_SIZE, list(BASE_HANDLE), q0)]   # 닫으면 위 손잡이와 포개짐
        for i, y in enumerate(MOLD_YS):
            parts += [
                (f"mold{i}_bottom", [sx, sy, WALL], [x, y, z0 + WALL / 2], q0),
                (f"mold{i}_wx0", [WALL, sy, sz], [x - sx / 2 + WALL / 2, y, z0 + sz / 2], q0),
                (f"mold{i}_wx1", [WALL, sy, sz], [x + sx / 2 - WALL / 2, y, z0 + sz / 2], q0),
                (f"mold{i}_wy0", [sx, WALL, sz], [x, y - sy / 2 + WALL / 2, z0 + sz / 2], q0),
                (f"mold{i}_wy1", [sx, WALL, sz], [x, y + sy / 2 - WALL / 2, z0 + sz / 2], q0),
            ]
        return parts

    def put(self, parts):
        for pid, size, ctr, q in parts:
            self.mbox(pid, size, ctr, q)

    def set_lid(self, phi):
        """뚜껑(위 철판 전체)을 phi도로 다시 그림"""
        for pid, size, ctr, q in self.lid_parts(phi):
            self.mbox(pid, size, ctr, q, wait=0.01)

    def closed_parts(self):
        """닫힌 철판 전체 (뒤집기 전 기준)"""
        return self.base_parts() + self.lid_parts(LID_CLOSED)

    def set_flip(self, theta):
        """닫힌 철판 전체를 손잡이 축(앞뒤 x 방향, y = STEEL_YC, z = AXIS_Z) 기준으로 theta도 돌린 모습으로 다시 그림"""
        t = math.radians(theta)
        c, s = math.cos(t), math.sin(t)
        qr = [math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)]
        for pid, size, ctr, q in self.closed_parts():
            dy, dz = ctr[1] - STEEL_YC, ctr[2] - AXIS_Z
            new = [ctr[0], STEEL_YC + dy * c - dz * s, AXIS_Z + dy * s + dz * c]
            self.mbox(pid, size, new, qmul(qr, list(q)), wait=0.01)

    def flip_all(self, back):
        """포개진 손잡이 2장(위 철판 손잡이 + 아래 철판 손잡이, 두께 합 2 * HANDLE_T)을 위아래로 한 번에 잡고
        틀 전체를 flip_lift만큼 들어 올림 → 공중에서 손목만 180도 → 제자리에 내려놓고 놓음.
        들고 있는 동안 틀 전체를 그리퍼에 붙여서 팔과 같이 움직이고 충돌 검사도 같이 받음.
        뚜껑을 닫은 직후면(back=False) 홈으로 가지 않고 그 자리에서 바로 고쳐 잡음.
        (뒤집혀도 손잡이 2장은 같은 자리에 위아래만 바뀌어 포개짐 → 되돌릴 때 같은 자세로 잡음)"""
        self.log(f"   ▶ 손잡이 2장({GRIP_FLIP * 100:.0f}cm) 한 번에 잡고 들어 올려 공중에서 "
                 f"{'되돌리기' if back else '180도 뒤집기'} (틀 {len(MOLD_YS)}개)")
        self.yaw = FLIP_YAW
        g = list(FLIP_GRIP_W)
        pre = add(g, self.dirvec(SIDE), -P["approach"])
        self.go(pre, SIDE, FLIP_ROLL0, j4_center=True)
        self.lin(g)
        ids = [pid for pid, *_ in self.closed_parts()]
        self.grab(ids, GRIP_FLIP)                             # 손가락이 위아래로 손잡이 2장을 함께 조임
        self.wait_attached(ids, True)
        hi = up(g, P["flip_lift"])
        self.log(f"      {P['flip_lift'] * 100:.0f}cm 들어 올리기")
        self.lin(hi)
        d = -180.0 if self.current_joints()[5] > 0 else 180.0
        start = 180.0 if back else 0.0
        n = max(1, int(round(180.0 / P["flip_step"])))
        for k in range(1, n + 1):                             # 붙어 있는 틀이 손목과 같이 돎
            self.straight((hi, SIDE, FLIP_ROLL0 + d * k / n))
            if k % 6 == 0 or k == n:
                self.log(f"      공중에서 {abs(d * k / n):.0f}도")
        self.log("      제자리에 내려놓기")
        self.lin(g)
        self.release(ids)
        self.wait_attached(ids, False)
        self.set_flip(start + d)                              # 놓은 자리를 계산값으로 맞춤 (손이 손잡이 축을 보고 roll +d → 철판도 +d)
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

    def scene_ids(self):
        """MoveIt 장면의 (바닥에 있는 물체 이름, 로봇에 붙어 있는 물체 이름)"""
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        fut = self.scene_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return set(), set()
        sc = fut.result().scene
        return ({o.id for o in sc.world.collision_objects},
                {a.object.id for a in sc.robot_state.attached_collision_objects})

    def apply_scene(self, ps):
        """장면 변경을 서비스로 확실하게 적용 (토픽은 막 켜진 직후 메시지가 사라질 수 있음)"""
        self.apply_cli.wait_for_service(timeout_sec=5.0)
        fut = self.apply_cli.call_async(ApplyPlanningScene.Request(scene=ps))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.02)
        return fut.done() and fut.result() is not None and fut.result().success

    def clear_scene(self):
        """예전 실행에서 남은 물체 지우기 (실패하면서 그리퍼에 붙은 채 남은 것 포함)"""
        world, attached = self.scene_ids()
        if attached:                                   # 1) 로봇에서 떼기 → 바닥 물체가 됨
            ps = PlanningScene(is_diff=True)
            ps.robot_state.is_diff = True
            for i in attached:
                ps.robot_state.attached_collision_objects.append(AttachedCollisionObject(
                    link_name=EE_LINK, object=CollisionObject(id=i, operation=CollisionObject.REMOVE)))
            self.apply_scene(ps)
            world, still = self.scene_ids()
            if still:
                self.warn(f"그리퍼에서 안 떨어진 물체: {', '.join(sorted(still))}")
        if world:                                      # 2) 바닥 물체 전부 지우기
            ps = PlanningScene(is_diff=True)
            for i in world:
                co = CollisionObject(id=i, operation=CollisionObject.REMOVE)
                co.header.frame_id = BASE_LINK
                ps.world.collision_objects.append(co)
            self.apply_scene(ps)
        if world or attached:
            self.log(f"🧹 남아 있던 물체 {len(world | attached)}개 지움 (붙어 있던 것 {len(attached)}개)")
        self.added = {}

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
        if ROBOT_PLATE:                                                   # 로봇 바닥판 (윗면은 base_link 4mm 아래)
            top = -0.004                                                  # 작업대 윗면(TZ) ~ top
            self.box("robot_plate", [ROBOT_PLATE[0], ROBOT_PLATE[1], top - TZ], [0.0, 0.0, (TZ + top) / 2])
        self.mbox("machine", MACHINE_SIZE, MACHINE_POS)
        # 옆 기둥·회전 손잡이 없음: 뒤집기는 틀 앞 손잡이 2개를 잡고 손잡이 축으로 돌림
        self.put(self.base_parts())
        self.put(self.lid_parts(LID_OPEN))                                # 처음: 뚜껑 열린 상태 (누움 또는 세움)
        self.cyl("kettle", KETTLE_H, KETTLE_R, [KX, KY, KZ])
        top = TZ + GAP + KETTLE_H
        self.box("kettle_handle", [0.10, 0.02, 0.02], KETTLE_GRIP)
        self.box("kettle_post1", [0.01, 0.02, 0.05], [KX - 0.045, KY, top + 0.025])
        self.box("kettle_post2", [0.01, 0.02, 0.05], [KX + 0.045, KY, top + 0.025])
        self.box("kettle_spout", [SPOUT_LEN, 0.03, 0.03], [KX + KETTLE_R + SPOUT_LEN / 2, KY, KZ + 0.04])
        for kind in SAUCES:
            g = sauce_grip(kind)
            sid, nid = sauce_parts(kind)
            if SAUCE_STAND_H > 0:                                         # 받침대
                self.box(f"{sid}_stand", [0.09, 0.09, SAUCE_STAND_H], [g[0], g[1], TZ + SAUCE_STAND_H / 2])
            self.cyl(sid, SAUCE_H, SAUCE_R, g)
            self.cyl(nid, NOZZLE_H, 0.008, [g[0], g[1], SAUCE_Z0 + SAUCE_H + NOZZLE_H / 2])
        self.box("plate", PLATE_SIZE, [PLATE_XY[0], PLATE_XY[1], TZ + GAP + PLATE_SIZE[2] / 2])
        time.sleep(1.0)

    def clear_plate(self):
        for fid in self.on_plate:
            self.m.remove_collision_object(id=fid)
            self.added.pop(fid, None)
            time.sleep(0.05)
        self.on_plate = []

    def pour_tcp(self, i):
        """i번 틀에 부을 때 주전자 손잡이(tcp) 위치. 주둥이는 항상 +x 방향"""
        tx, ty = mold_w(i, POUR_X)
        tx += P.get("pour_dx", 0.0)
        return [tx - SPOUT_TIP[0], ty, MOLD_TOP + P["pour_clear"] - SPOUT_TIP[1]]

    # ---------- 공정 단위 ----------
    def pick_kettle(self):
        self.go(up(KETTLE_GRIP, P["approach"]))
        self.lin(KETTLE_GRIP)
        self.grab(KETTLE_PARTS, GRIP_KETTLE)
        self.lin(up(KETTLE_GRIP, P["kettle_lift"]))

    def pour_all(self, hold, molds):
        t = self.pour_tcp(molds[0])
        self.hv(t)
        for k, i in enumerate(molds):
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀")
            tip = [t[0] + SPOUT_TIP[0], t[1], t[2] + SPOUT_TIP[1]]   # 주둥이 끝 = 틀 2/3 지점 위 pour_clear
            self.tilt_tip(tip, 0.0, P["pour_tilt"])
            self.wait(hold, "붓기")
            self.tilt_tip(tip, P["pour_tilt"], 0.0)
            self.lin(up(t, P["pour_hop"]))
            if k == len(molds) - 1:
                return
            nxt = self.pour_tcp(molds[k + 1])
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
        self.yaw = SAUCE_GRASP_YAW[kind]
        pre = add(grip, self.dirvec(SIDE), -P["approach"])
        self.go(pre, SIDE, SAUCE_GRASP_ROLL[kind])
        self.lin(grip)
        self.grab(parts, GRIP_SAUCE)

        self.set_step(f"   [{kind}] 입구 위로 들어올려 {MOLD_NAMES[molds[0]]} 틀 위로 → 입구를 틀 쪽으로")
        z_d = MOLD_TOP + P["nozzle_gap"] + NOZZLE_TIP
        S = {i: mold_w(i, MOLD_X0 + P["dispense_front"]) for i in molds}   # 짜기 시작점 (틀 손잡이 쪽, 실제 좌표)
        E = {i: mold_w(i, MOLD_X1 - P["dispense_back"]) for i in molds}    # 짜기 끝점 (경첩 쪽)
        s0 = S[molds[0]]
        carry = SAUCE_CARRY_Z                                            # 들고 이동하는 높이
        self.lin([grip[0], grip[1], carry])                              # 들어올리기
        xt = SAUCE_TURN_X                                                # 손 방향 돌리는 x 위치
        self.lin([xt, grip[1], carry])                                   # 먼저 로봇 쪽으로 빠지고 (옆 소스통 위로 안 지나가게)
        self.lin([xt, s0[1], carry])                                     # 잡은 방향 그대로 첫 틀 줄로 (수평)
        self.turn_yaw(SAUCE_YAW)                                         # 거기서 손을 옆으로
        self.hv([s0[0], s0[1], z_d + P["flip_clear"]], travel_z=carry)
        self.flip_sauce(down=True)

        self.set_step(f"   [{kind}] 속재료 넣기: {names} (각 손잡이 쪽 → 경첩 쪽)")
        for k, i in enumerate(molds):
            a, b = S[i], E[i]
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀")
            if k > 0:
                self.lin([a[0], a[1], z_d + P["dispense_hop"]])
            self.lin([a[0], a[1], z_d])
            self.gripper(GRIP_SQUEEZE, "더 조이기 (짜기 시작)")
            self.lin([b[0], b[1], z_d], speed=P["dispense_speed"])
            self.gripper(GRIP_SAUCE, "조임 풀기 (짜기 멈춤)")
            self.lin([b[0], b[1], z_d + P["dispense_hop"]])

        self.set_step(f"   [{kind}] 입구를 다시 위로 → 제자리 → 내려놓기")
        c = self.cur[0]
        sl = S[molds[-1]]
        self.lin([c[0], c[1], z_d + P["flip_clear"]])    # 수직으로 올라가서
        self.lin([sl[0], sl[1], z_d + P["flip_clear"]])  # 뚜껑(경첩)에서 먼 손잡이 쪽으로 수평 이동
        self.flip_sauce(down=False)                       # 거기서 입구를 위로
        self.lin([sl[0], sl[1], carry])
        self.lin([xt, sl[1], carry])
        self.turn_yaw(SAUCE_GRASP_YAW[kind])              # 경첩 너머에서 다시 잡을 때 방향으로
        self.lin([xt, grip[1], carry])                    # 제 줄로 옮긴 뒤
        self.lin([grip[0], grip[1], carry])
        self.lin(grip)
        self.release(parts)
        self.color(parts, FILL_COLORS[kind])
        self.lin(pre)
        self.yaw = 0.0

    def lid_pitch(self, phi):
        """뚜껑 각도별 그리퍼 기울기: 열림(멀리) lid_pitch → 닫힘(가까이) 0도"""
        return P["lid_pitch"] * (LID_CLOSED - phi) / (LID_CLOSED - LID_OPEN)

    def move_lid(self, phi0, phi1):
        """위 철판 손잡이를 잡은 채 경첩 중심으로 phi0 → phi1 (뚜껑 전부 한 번에)"""
        n = max(1, int(round(abs(phi1 - phi0) / P["lid_step"])))
        for k in range(1, n + 1):
            phi = phi0 + (phi1 - phi0) * k / n
            self.straight((self.handle_w(phi), self.lid_pitch(phi), 0.0))
            self.set_lid(phi)
            if k % 6 == 0 or k == n:
                self.log(f"      뚜껑 {abs(phi - phi0):.0f}도")

    def lid_closed_seed(self):
        """열린 손잡이에서 닫힌 손잡이까지 이어지는 관절 해를 따라가서, 닫힌 쪽 끝 자세를 돌려줌.
        열 때 이 해로 잡아야 끝(완전히 누움)까지 이어짐 (다른 해로 잡으면 중간에 끊길 수 있음)"""
        q = self.ik_pref(*self.flange(self.handle_w(LID_OPEN), self.lid_pitch(LID_OPEN)))
        if q is None:
            return None
        n = max(1, int(round((LID_CLOSED - LID_OPEN) / P["lid_step"])))
        for k in range(1, n + 1):
            phi = LID_OPEN + (LID_CLOSED - LID_OPEN) * k / n
            qn = self.ik_near(*self.flange(self.handle_w(phi), self.lid_pitch(phi)), q)
            if qn is None or self.jump(q, qn):
                return None
            q = qn
        return q

    def lid(self, close):
        """철판 뚜껑 한 번에 닫기(열림 → 180도 → 닫힘) / 열기(반대로)"""
        p0, p1 = (LID_OPEN, LID_CLOSED) if close else (LID_CLOSED, LID_OPEN)
        self.yaw = MACHINE_YAW                          # 손 방향을 기계 방향에 맞춤 (손가락이 경첩 축 방향)
        g = self.handle_w(p0)
        above = up(g, P["approach"])
        q_up = None
        if not close:                                   # 열기: 끝까지 이어지는 해로 손잡이 위에 가기
            seed = self.lid_closed_seed()
            if seed is not None:
                q_up = self.ik_near(*self.flange(above, self.lid_pitch(p0)), seed)
            if q_up is not None and not self.path_clear(self.current_joints(), q_up):
                q_up = None
            if q_up is None:
                self.warn(f"[{self.step}] 끝까지 이어지는 열기 자세를 못 찾음 → 보통 방법으로 접근")
        if q_up is not None:
            self.joints(q_up, checked=True)
            self.cur = (above, self.lid_pitch(p0), 0.0)
            self.cur_yaw = self.yaw
        else:
            self.go(above, self.lid_pitch(p0))
        self.lin(g)
        self.gripper(GRIP_LID, "CLOSE (철판 손잡이 조이기)")
        self.move_lid(p0, p1)
        self.gripper(GRIP_OPEN, "OPEN (풀기)")
        self.lin(up(self.handle_w(p1), P["approach"]))
        self.yaw = 0.0

    # ---------- 한 판 굽기 (틀 개수만큼) ----------
    def bake_batch(self, fillings, tag):
        molds = list(range(len(fillings)))
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
        self.home()                                    # 소스통 내려놓은 자세에서 바로 가면 손목 뒤집힌 해가 잡힘
        self.pick_kettle()
        self.set_step(f"{tag} 6. 2차 반죽")
        self.pour_all(P["pour2_hold"], molds)
        high_z = self.pour_tcp(molds[-1])[2] + P["pour2_lift"]
        self.set_step(f"{tag} 7. {P['pour2_lift'] * 100:.0f}cm 위로 → 주전자 제자리")
        self.place_kettle(high_z)

        self.set_step(f"{tag} 8. 철판 뚜껑 한 번에 닫기 ({LID_CLOSED - LID_OPEN:.0f}도)")
        self.lid(close=True)
        if P["bake_front_s"] > 0:
            self.set_step(f"{tag} 9. 윗면 굽기")
            self.wait(P["bake_front_s"], "윗면 굽기")
        self.set_step(f"{tag} 9-1. 닫은 자리에서 바로 위·아래 손잡이 함께 잡고 들어 올려 공중에서 180도 뒤집기")
        self.flip_all(back=False)                      # 홈으로 안 가고 뚜껑 닫은 손으로 바로 고쳐 잡음
        self.home()
        self.set_step(f"{tag} 9-2. 뒤집은 채로 굽기")
        self.wait(P["bake_back_s"], "굽기")
        self.set_step(f"{tag} 9-3. 위·아래 손잡이 함께 잡고 들어 올려 공중에서 180도 되돌리기")
        self.flip_all(back=True)
        self.home()

        self.set_step(f"{tag} 10. 철판 뚜껑 한 번에 열기 ({LID_CLOSED - LID_OPEN:.0f}도)")
        self.lid(close=False)
        self.wait(P["open_wait"], "식히기")

        self.set_step(f"{tag} 11. 붕어빵 꺼내서 접시에 놓기")
        for i in molds:
            n = len(self.on_plate)
            fid = f"fish_{n}"
            self.log(f"   ▶ {MOLD_NAMES[i]} 틀 ({fillings[i]}) → 접시 {n + 1}번 자리")
            fish = mold_w(i) + [MOLD_BASE + WALL + GAP + FISH_SIZE[2] / 2]
            self.box(fid, FISH_SIZE, fish)
            self.color([fid], FILL_COLORS[fillings[i]])
            time.sleep(0.3)
            self.go(up(fish, P["approach"] + MOLD_D))
            self.lin(fish)
            self.grab([fid], GRIP_FISH)
            dx, dy, lv = PLATE_SPOTS[n]
            spot = [PLATE_XY[0] + dx, PLATE_XY[1] + dy,
                    TZ + GAP + PLATE_SIZE[2] + GAP + FISH_SIZE[2] / 2 + lv * (FISH_SIZE[2] + GAP)]
            tz = fish[2] + P["fish_lift"]
            if PLATE_VIA:
                self.hv([PLATE_VIA[0], PLATE_VIA[1], tz])
            self.hv(spot, travel_z=tz)
            self.release([fid])
            self.color([fid], FILL_COLORS[fillings[i]])
            self.on_plate.append(fid)
            self.lin(up(spot, P["approach"]))
        self.home()

    def serve(self, fills):
        n = len(MOLD_YS)
        batches = [fills[i:i + n] for i in range(0, len(fills), n)]
        for b, chunk in enumerate(batches, 1):
            self.set_step(f"🔥 {b}/{len(batches)}판 굽기 시작: " + ", ".join(chunk))
            self.bake_batch(chunk, f"[{b}판]")
        self.set_step(f"🐟 붕어빵 {len(fills)}개 완성! (팥 {fills.count('팥')}, 슈크림 {fills.count('슈크림')})")

# repeated 
    def shop(self):
        """가게 운영: 손님 대기 → 90도 인사 → 주문(끄덕/도리도리) → 굽기 → 90도 인사 → 반복"""
        self.log(f"🤖 로봇 모델 {MODEL} 배치 사용 (속도 vel {P['vel']})")
        self.wait_scene_ready()
        self.clear_scene()
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
    node = Node("bungeoppang_pour_demo")
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
