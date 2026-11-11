"""RViz 표시용 노드
1) /bungeoppang/gripper_width(mm)를 받아 그리퍼 손가락 관절값(/gripper/joint_states)으로 보냄
2) 작업대·안전 한계·최대 반경 + 물체(붕어빵 기계, 생선 칸 2개, 주전자, 소스통, 끈 고리)를
   마커(/bungeoppang/scene)로 표시 — 위치는 positions.py의 티칭 좌표에서 자동 계산
※ 물체 마커는 '놓여 있는 자리' 표시용이라 로봇이 집어도 따라 움직이지는 않아요.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from bungeoppang2_yammy import workcell as W

# ---- 물체 표시 크기 (mm) ★ 실물에 맞게 수정 ----
KETTLE_DIA, KETTLE_H = 120.0, 150.0   # ★ 주전자 몸통 지름·높이
SPOUT_DIA, SPOUT_LEN = 25.0, 100.0    # ★ 주전자 주둥이
SAUCE_H = W.SAUCE_H - 30.0             # 소스통 몸통 (전체 높이에서 노즐 3cm 제외)
NECK_DIA, NECK_H = 18.0, 30.0         # ★ 소스통 입구(노즐)
MOLD_THICK = 40.0                      # ★ 틀 두께 (위아래 판 합)
BOX_SHRINK = 10.0                      # ★ 틀 아래 상자: 틀보다 얼마나 작게

FINGER_JOINTS = ["rh_r1", "rh_r2", "rh_l1", "rh_l2"]
Q_CLOSED = 1.1        # 모델에서 완전히 닫혔을 때 관절값 (rad)
STROKE_MM = 109.0


class GripperViz(Node):
    def __init__(self):
        super().__init__("gripper_viz")
        self.width = W.GRIP_OPEN_WIDTH
        self.create_subscription(Float64, "/bungeoppang/gripper_width", self.on_width, 10)
        self.js_pub = self.create_publisher(JointState, "/gripper/joint_states", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "/bungeoppang/scene", 10)
        self.create_timer(0.05, self.publish_joints)
        self.create_timer(1.0, self.publish_scene)
        self.held = ""
        self.held_flange = None
        self.cup_fish = False        # 손님 컵에 붕어빵이 들어 있는지 (표시용)
        self.create_subscription(String, "/bungeoppang/held", self.on_held, 10)

    def on_width(self, msg):
        self.width = max(0.0, min(STROKE_MM, msg.data))

    def publish_joints(self):
        q = Q_CLOSED * (1.0 - self.width / STROKE_MM)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = FINGER_JOINTS
        js.position = [q] * 4
        self.js_pub.publish(js)

    def _box(self, mid, cx, cy, cz, sx, sy, sz, rgba):
        m = Marker()
        m.header.frame_id = "base_link"
        m.ns, m.id, m.type, m.action = "scene", mid, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx / 1000, cy / 1000, cz / 1000
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = sx / 1000, sy / 1000, sz / 1000
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _cyl(self, mid, cx, cy, cz, dia, h, rgba, quat=None):
        m = Marker()
        m.header.frame_id = "base_link"
        m.ns, m.id, m.type, m.action = "scene", mid, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx / 1000, cy / 1000, cz / 1000
        if quat:
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        else:
            m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = dia / 1000
        m.scale.z = h / 1000
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _ecyl(self, mid, cx, cy, cz, sx, sy, h, rgba):
        """가로·세로 지름이 다른 타원 기둥 (생선 칸 홈)"""
        m = self._cyl(mid, cx, cy, cz, sx, h, rgba)
        m.scale.y = sy / 1000
        return m

    def _text(self, mid, cx, cy, cz, text, size=40.0):
        m = Marker()
        m.header.frame_id = "base_link"
        m.ns, m.id, m.type, m.action = "scene", mid, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx / 1000, cy / 1000, cz / 1000
        m.pose.orientation.w = 1.0
        m.scale.z = size / 1000
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        return m

    @staticmethod
    def _quat_z_to(d):
        """원통의 축(z)을 방향 d(수평 단위벡터)로 눕히는 쿼터니언"""
        ax, ay = -d[1], d[0]          # 회전축 = z × d
        n = math.hypot(ax, ay) or 1.0
        s = math.sin(math.pi / 4)     # 90° 회전
        return (ax / n * s, ay / n * s, 0.0, math.cos(math.pi / 4))

    # ---- 틀(뚜껑·전체)을 잡고 돌리면 RViz 그림도 같이 움직이고, 놓은 자리에 그대로 남게 ----
    LID_FAR = {71, 73, 25, 27, 33, 35, 91, 93}
    LID_NEAR = {70, 72, 24, 26, 32, 34, 90, 92}
    MOLD_ALL = LID_FAR | LID_NEAR | {29}

    def _T(self, f):
        R = self._rot_zyz(*f[3:6])
        return R, [f[0] / 1000, f[1] / 1000, f[2] / 1000]

    @staticmethod
    def _compose(A, B):
        Ra, ta = A; Rb, tb = B
        R = [[sum(Ra[i][k] * Rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        t = [sum(Ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
        return R, t

    @staticmethod
    def _inv(A):
        R, t = A
        Rt = [[R[j][i] for j in range(3)] for i in range(3)]
        return Rt, [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]

    def _apply(self, m, T):
        R, t = T
        p = m.pose.position
        q = [sum(R[i][k] * v for k, v in enumerate((p.x, p.y, p.z))) + t[i] for i in range(3)]
        o = m.pose.orientation
        M = self._quat_to_mat((o.x, o.y, o.z, o.w))
        M2 = [[sum(R[i][k] * M[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = q
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = self._mat_to_quat(M2)

    def _mold_track(self, data):
        name, _, fl = data.partition("|")
        fl = [float(x) for x in fl.split(",")] if fl else None
        if not hasattr(self, "mold_T"):
            self.mold_T, self.mold_grab = {}, None
        if name == "reset":
            self.mold_T, self.mold_grab = {}, None
            return True
        groups = {"lid": self.LID_FAR, "top": self.LID_NEAR, "mold": self.MOLD_ALL | {75}, "near": self.LID_NEAR, "far": self.LID_FAR, "turn": self.MOLD_ALL | {74, 75}}
        if name in groups and fl:
            self.mold_grab = (groups[name], fl)
            return True
        if name == "" and self.mold_grab is not None:
            ids, fg = self.mold_grab
            if fl:
                rel = self._compose(self._T(fl), self._inv(self._T(fg)))
                for i in ids:
                    self.mold_T[i] = self._compose(rel, self.mold_T.get(i, ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])))
            self.mold_grab = None
            return True
        return False

    def on_held(self, msg):
        if self._mold_track(msg.data):
            self.held, self.held_flange = "", None
            self.publish_scene()
            return
        name, _, fl = msg.data.partition("|")
        if name.startswith("cup:"):                 # 붕어빵이 손님 컵에 들어감 (꼬챙이는 계속 들고 있음)
            self.cup_fish = True
            self.publish_scene()
            return
        if name == "skewer":
            self.cup_fish = False
        self.held = name
        self.held_flange = [float(v) for v in fl.split(",")] if fl else None
        self.publish_scene()

    def _held_group(self):
        if self.held == "kettle":
            return {8, 9, 50, 51, 52}, W.KETTLE_PICK
        if self.held == "skewer":
            return {100, 101}, W.skewer_rest_pose()
        if self.held.startswith("sauce:"):
            kind = self.held.split(":", 1)[1]
            j = 0 if kind == "팥" else 1
            return {12 + j * 3, 13 + j * 3, 14 + j * 3}, W.SAUCE_PICKS.get(kind, W.SAUCE_PICK)
        return set(), None

    @staticmethod
    def _rot_zyz(a, b, c):
        a, b, c = (math.radians(t) for t in (a, b, c))
        ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
        return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
                [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
                [-sb * cc, sb * sc, cb]]

    @staticmethod
    def _mat_to_quat(m):
        tr = m[0][0] + m[1][1] + m[2][2]
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
        if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
            return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
        if m[1][1] > m[2][2]:
            s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
            return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)

    @staticmethod
    def _quat_to_mat(q):
        x, y, z, w = q
        return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

    def _attach(self, m, flange):
        R = self._rot_zyz(*flange[3:6])
        t = [flange[0] / 1000, flange[1] / 1000, flange[2] / 1000]
        p = m.pose.position
        d = [p.x - t[0], p.y - t[1], p.z - t[2]]
        loc = [sum(R[r][k] * d[r] for r in range(3)) for k in range(3)]
        o = m.pose.orientation
        Mw = self._quat_to_mat((o.x, o.y, o.z, o.w))
        Ml = [[sum(R[r][i] * Mw[r][j] for r in range(3)) for j in range(3)] for i in range(3)]
        q = self._mat_to_quat(Ml)
        m.header.frame_id = "link_6"
        m.frame_locked = True
        m.pose.position.x, m.pose.position.y, m.pose.position.z = loc
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q

    def publish_scene(self):
        arr = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = "base_link"
        wipe.ns = "scene_wipe"      # 다른 마커(id 0)와 이름이 겹치지 않게 별도 ns
        wipe.id = 9999
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        s = W.MOLD_SIGN
        cx = s * (W.TABLE_FWD - W.TABLE_BACK) / 2         # 작업대 중심 (틀 방향 = X)
        cy = W.SHORT_SIGN * (W.TABLE_SHORT_A - W.TABLE_SHORT_B) / 2
        # 로봇 받침대 (17 x 21 x 7cm, 베이스 바로 아래)
        arr.markers.append(self._box(60, 0.0, 0.0, -W.PEDESTAL_H / 2,
                           W.PEDESTAL_LONG, W.PEDESTAL_SHORT, W.PEDESTAL_H,
                           (0.55, 0.55, 0.6, 1.0)))
        # 로봇 뒤 보조 테이블
        f0, f1 = W.TABLE2_FWD; s0, s1 = W.TABLE2_SHORT
        arr.markers.append(self._box(61, W.MOLD_SIGN * (f0 + f1) / 2, W.SHORT_SIGN * (s0 + s1) / 2,
                           W.TABLE_TOP_Z - 10, f1 - f0, s1 - s0, 20, (0.92, 0.92, 0.92, 0.7)))
        # 작업대 (두께 2cm)
        arr.markers.append(self._box(
            0, cx, cy, W.TABLE_TOP_Z - 10,
            W.TABLE_FWD + W.TABLE_BACK, W.TABLE_SHORT_A + W.TABLE_SHORT_B, 20,
            (0.80, 0.65, 0.45, 0.6)))
        # 틀 방향 안전 한계 (빨간 반투명 벽)
        arr.markers.append(self._box(
            1, s * W.SAFE_FWD_MAX, cy, W.TABLE_TOP_Z + 300,
            4, W.TABLE_SHORT_A + W.TABLE_SHORT_B, 600, (1.0, 0.1, 0.1, 0.25)))
        # 최대 반경 (주황 원)
        ring = Marker()
        ring.header.frame_id = "base_link"
        ring.ns, ring.id, ring.type, ring.action = "scene", 2, Marker.LINE_STRIP, Marker.ADD
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.005
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 0.6, 0.0, 0.9
        from geometry_msgs.msg import Point
        for i in range(73):
            t = math.radians(i * 5)
            ring.points.append(Point(x=W.SAFE_RADIUS_MAX / 1000 * math.cos(t),
                                     y=W.SAFE_RADIUS_MAX / 1000 * math.sin(t),
                                     z=(W.TABLE_TOP_Z + 2) / 1000))
        arr.markers.append(ring)

        # ---- 물체들 (positions.py 티칭 좌표 → 손끝 위치에서 계산) ----
        zdir = W.tool_dir(W.MOLD_GRIP)                 # 틀 방향 단위벡터
        tip = W.tool_tip(W.MOLD_GRIP)                  # 손잡이 잡는 점

        # 철판(석쇠 23x18) 위에 틀 2개(각 폭 9cm)가 나란히, 활짝 열린 상태.
        # 틀마다 가까운 판 + 먼 판(각 7.75cm), 판마다 생선 칸 1개(8x5cm). 손잡이는 철판 가운데 1개.
        tip = W.tool_tip(W.MOLD_GRIP)
        edge = [tip[k] + zdir[k] * W.HANDLE_STICK for k in range(3)]
        hinge = [edge[k] + zdir[k] * W.MOLD_W for k in range(3)]
        gx = W.GRILL_FWD if abs(zdir[0]) > 0.5 else W.GRILL_SHORT
        gy = W.GRILL_SHORT if abs(zdir[0]) > 0.5 else W.GRILL_FWD
        arr.markers.append(self._box(29, hinge[0], hinge[1], W.MOLD_TOP_Z - MOLD_THICK / 2 - 3,
                           gx, gy, 3, (0.75, 0.75, 0.78, 0.9)))             # 철판
        arr.markers.append(self._box(74, hinge[0], hinge[1], W.TABLE_TOP_Z + W.MOLD_FLOAT / 2,
                           gx - BOX_SHRINK, gy - BOX_SHRINK, W.MOLD_FLOAT,
                           (0.35, 0.35, 0.38, 0.95)))                       # 받침 상자
        for mi, g in enumerate(W.MOLD_GRIPS):           # 틀 1, 2
            ti = W.tool_tip(g)
            ei = [ti[k] + zdir[k] * W.HANDLE_STICK for k in range(2)]
            for pi in range(2):                          # 0 = 가까운 판, 1 = 먼 판
                c = [ei[0] + zdir[0] * W.MOLD_W * (0.5 + pi),
                     ei[1] + zdir[1] * W.MOLD_W * (0.5 + pi)]
                k = mi * 2 + pi
                arr.markers.append(self._box(70 + k, c[0], c[1], W.MOLD_TOP_Z - MOLD_THICK / 4,
                                   (W.MOLD_W if abs(zdir[0]) > 0.5 else W.MOLD_L) - 2,
                                   (W.MOLD_L if abs(zdir[0]) > 0.5 else W.MOLD_W) - 2,
                                   MOLD_THICK / 2, (0.35, 0.55, 0.95, 0.95)))
                fl, fw = (W.FISH_W, W.FISH_L) if abs(zdir[0]) > 0.5 else (W.FISH_L, W.FISH_W)
                arr.markers.append(self._box(24 + k, c[0], c[1], W.MOLD_TOP_Z + 2, fl, fw, 4,
                                   (1.0, 0.85, 0.3, 0.95)))             # 생선 칸 (8x5)
                arr.markers.append(self._box(32 + k, c[0], c[1],
                                   W.MOLD_TOP_Z + 4 - W.FISH_DEPTH / 2,
                                   fl - 2 * W.FISH_INSET, fw - 2 * W.FISH_INSET,
                                   W.FISH_DEPTH, (0.45, 0.33, 0.12, 1.0)))  # 반죽 홈
            lab = [ei[0] - zdir[0] * 25, ei[1] - zdir[1] * 25]
            arr.markers.append(self._text(78 + mi, lab[0], lab[1], W.MOLD_TOP_Z + 30, f"{mi + 1}번"))
        hl = (W.HANDLE_L, W.HANDLE_W) if abs(zdir[0]) > 0.5 else (W.HANDLE_W, W.HANDLE_L)
        hn = [edge[k] - zdir[k] * W.HANDLE_L / 2 for k in range(3)]
        arr.markers.append(self._box(22, hn[0], hn[1], tip[2], hl[0], hl[1], W.HANDLE_T,
                           (0.75, 0.55, 0.3, 0.95)))
        far_edge = [edge[k] + zdir[k] * 2 * W.MOLD_W for k in range(3)]
        hf = [far_edge[k] + zdir[k] * W.HANDLE_L / 2 for k in range(3)]
        arr.markers.append(self._box(23, hf[0], hf[1], tip[2], hl[0], hl[1], W.HANDLE_T,
                           (0.75, 0.55, 0.3, 0.95)))
        # 철판 옆 회전 손잡이 (회전축 = 짧은 방향, 로봇 중심선 쪽으로 튀어나옴)
        mid_s = (W.SIDE_HANDLE_BASE + W.SIDE_HANDLE_END) / 2
        arr.markers.append(self._cyl(75, W.MOLD_SIGN * W.SIDE_HANDLE_FWD, W.SHORT_SIGN * mid_s,
                           W.SIDE_HANDLE_Z, W.SIDE_HANDLE_T, W.SIDE_HANDLE_LEN,
                           (0.75, 0.55, 0.3, 0.95), quat=self._quat_z_to((0.0, 1.0))))
        far_grab = [far_edge[k] + zdir[k] * W.HANDLE_STICK for k in range(3)]
        arr.markers.append(self._cyl(28, far_grab[0], far_grab[1],
                           far_grab[2] - W.STRING_LEN, 15.0, 6.0, (0.2, 0.2, 0.2, 1.0)))  # 끈 고리

        # 비커(계량컵) — 9cm 상자 위: 투명한 몸통 + 주둥이 + 옆면 손잡이
        cx, cy = W.MOLD_SIGN * W.KETTLE_FWD, W.SHORT_SIGN * W.KETTLE_SHORT
        bz = W.CUP_BOTTOM_Z
        arr.markers.append(self._box(53, cx, cy, W.TABLE_TOP_Z + W.CUP_BASE_H / 2,
                           W.CUP_D + 30, W.CUP_D + 30, W.CUP_BASE_H, (0.45, 0.35, 0.25, 0.95)))
        gt = W.grip_point(W.KETTLE_PICK, W.CUP_GRIP_DEPTH)   # 손잡이 위치 (손끝보다 깊게 잡은 만큼 안쪽)
        hd = math.hypot(gt[0] - cx, gt[1] - cy)
        hx, hy = (gt[0] - cx) / hd, (gt[1] - cy) / hd
        arr.markers.append(self._cyl(8, cx, cy, bz + W.CUP_H / 2,
                           W.CUP_D, W.CUP_H, (0.85, 0.9, 0.95, 0.55)))
        sx, sy = cx - hx * (W.CUP_D / 2 + 6), cy - hy * (W.CUP_D / 2 + 6)
        arr.markers.append(self._box(9, sx, sy, bz + W.CUP_H - 5,
                           14 if abs(hx) > 0.5 else 22, 22 if abs(hx) > 0.5 else 14, 10,
                           (0.85, 0.9, 0.95, 0.8)))
        hz0, hz1 = bz + 15, bz + W.CUP_H - 5
        arr.markers.append(self._box(50, gt[0], gt[1], (hz0 + hz1) / 2, 12, 12, hz1 - hz0,
                           (0.8, 0.85, 0.9, 0.9)))
        ln = W.CUP_HANDLE_REACH - W.CUP_D / 2
        mx, my = (gt[0] + cx + hx * W.CUP_D / 2) / 2, (gt[1] + cy + hy * W.CUP_D / 2) / 2
        for k, z in ((51, hz1), (52, hz0)):
            arr.markers.append(self._box(k, mx, my, z, ln if abs(hx) > 0.5 else 12,
                               12 if abs(hx) > 0.5 else ln, 10, (0.8, 0.85, 0.9, 0.9)))

        # 소스통 2개: 팥(빨강) / 슈크림(크림색), 몸통 + 입구 + 이름표
        for j, (pose, color, name) in enumerate([
                (W.SAUCE_PICK_REDBEAN, (0.72, 0.15, 0.15, 0.95), "팥"),
                (W.SAUCE_PICK_CREAM, (1.0, 0.92, 0.65, 0.95), "슈크림")]):
            st = W.grip_point(pose, W.SAUCE_GRIP_DEPTH)       # 통 중심
            base = 12 + j * 3
            arr.markers.append(self._cyl(base, st[0], st[1], W.TABLE_TOP_Z + SAUCE_H / 2,
                               W.SAUCE_DIA, SAUCE_H, color))
            arr.markers.append(self._cyl(base + 1, st[0], st[1],
                               W.TABLE_TOP_Z + SAUCE_H + NECK_H / 2,
                               NECK_DIA, NECK_H, (0.9, 0.9, 0.9, 0.95)))
            arr.markers.append(self._text(base + 2, st[0], st[1],
                               W.TABLE_TOP_Z + SAUCE_H + NECK_H + 45, name))

        # 꼬챙이 꽂이 + 눕혀 꽂힌 꼬챙이 (막대는 -짧은방향으로 꽂이 안, 손잡이는 +짧은방향)
        q_y = self._quat_z_to((0.0, 1.0))
        brown, steel, fishc = (0.55, 0.35, 0.2, 1.0), (0.8, 0.8, 0.85, 1.0), (0.85, 0.6, 0.25, 1.0)
        f, s = W.SKEWER_SLOT                    # 꼬챙이는 1개
        x = W.MOLD_SIGN * f
        hold_c = s - W.SKEWER_HANDLE_L / 2 - W.SKEWER_ROD + W.SKEWER_IN_HOLDER / 2
        hz = W.SKEWER_REST_Z + 15 - W.TABLE_TOP_Z                      # 꽂이 블록 높이
        arr.markers.append(self._box(120, x, W.SHORT_SIGN * hold_c, W.TABLE_TOP_Z + hz / 2,
                           30, W.SKEWER_IN_HOLDER, hz, (0.4, 0.4, 0.45, 0.9)))
        arr.markers.append(self._cyl(100, x, W.SHORT_SIGN * s, W.SKEWER_REST_Z,
                           W.SKEWER_HANDLE_D, W.SKEWER_HANDLE_L, brown, quat=q_y))
        arr.markers.append(self._cyl(101, x, W.SHORT_SIGN * (s - W.SKEWER_HANDLE_L / 2 - W.SKEWER_ROD / 2),
                           W.SKEWER_REST_Z, W.SKEWER_ROD_D, W.SKEWER_ROD, steel, quat=q_y))
        # 손님이 들고 있는 종이컵 (+ 마지막으로 넣은 붕어빵)
        ccx, ccy = W.cup_center()
        arr.markers.append(self._cyl(130, ccx, ccy, W.CUP_TOP_Z - W.PAPER_CUP_H / 2, W.PAPER_CUP_D,
                           W.PAPER_CUP_H, (0.97, 0.97, 0.95, 0.45)))
        arr.markers.append(self._text(131, ccx, ccy, W.CUP_TOP_Z + 40, "손님 컵", size=30.0))
        if self.cup_fish:
            arr.markers.append(self._box(132, ccx, ccy,
                               W.CUP_TOP_Z - W.PAPER_CUP_H + W.CUP_FISH_GAP + W.FISH_L / 2,
                               W.FISH_W, 20, W.FISH_L, fishc))

        # 틀마다 손잡이 2개 (가까운 판 → 로봇 쪽, 먼 판 → 바깥쪽). 철판 가운데 손잡이·끈 고리는 지움
        arr.markers = [m for m in arr.markers if not (m.ns == "scene" and m.id in (22, 23, 28))]
        zd = W.tool_dir(W.MOLD_GRIP)
        hl = (W.HANDLE_L, W.HANDLE_W) if abs(zd[0]) > 0.5 else (W.HANDLE_W, W.HANDLE_L)
        for mi, hs_ in enumerate(W.HANDLE_SHORTS):
            ti = W.tool_tip(W.pose_to(W.MOLD_FWD, hs_, W.TABLE_TOP_Z + W.MOLD_FLOAT))
            e0 = [ti[k] + zd[k] * W.HANDLE_STICK for k in range(3)]
            e1 = [e0[k] + zd[k] * 2 * W.MOLD_W for k in range(3)]
            for k2, (e, sg) in enumerate(((e0, -1), (e1, +1))):
                c = [e[k] + sg * zd[k] * W.HANDLE_L / 2 for k in range(3)]
                arr.markers.append(self._box(90 + mi * 2 + k2, c[0], c[1], ti[2], hl[0], hl[1],
                                   W.HANDLE_T, (0.75, 0.55, 0.3, 0.95)))
        # 경첩 원기둥 봉 삭제: 경첩 줄 위에 있는 원기둥 마커는 그리지 않음
        hx = W.MOLD_SIGN * W.HINGE_FWD / 1000.0
        arr.markers = [m for m in arr.markers
                       if not (m.type == Marker.CYLINDER and m.header.frame_id == "base_link" and m.id != 75
                               and abs(m.pose.position.x - hx) < 0.04
                               and abs(m.pose.position.z * 1000 - (W.TABLE_TOP_Z + W.MOLD_FLOAT)) < 80)]
        # 경첩 봉 (돌림판 돌리는 용도): 짧은 방향으로, 끝이 철판 밖으로 나옴
        cx_, cy_ = W.MOLD_SIGN * W.HINGE_FWD, W.SHORT_SIGN * W.MOLD_SHORT
        q_ = self._quat_z_to((0.0, 1.0, 0.0))
        arr.markers.append(self._cyl(75, cx_, cy_, W.HINGE_Z, W.ROD_DIA, 2 * W.ROD_R + 20, (0.6, 0.6, 0.62, 1.0), quat=q_))
        mT = getattr(self, "mold_T", {}); mg = getattr(self, "mold_grab", None)
        for m in arr.markers:
            if m.ns != "scene" or m.header.frame_id != "base_link":
                continue
            if m.id in mT:
                self._apply(m, mT[m.id])
            if mg and m.id in mg[0]:
                self._attach(m, mg[1])
        ids, flange = self._held_group()
        if flange is not None and self.held_flange:
            flange = self.held_flange
        if flange is not None:
            for m in arr.markers:
                if m.ns == "scene" and m.id in ids:
                    self._attach(m, flange)
        self.mk_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GripperViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
