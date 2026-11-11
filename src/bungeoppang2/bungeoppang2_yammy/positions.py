"""★★ 위치 · 물체 길이 전용 파일 ★★

작업대, 안전 영역, 물체 치수, 티칭 좌표, 속도를 모아 뒀어요.
그리퍼 힘·폭 값은 grip_params.py에 있어요. 단위는 mm와 도(deg).

[좌표 약속 — RViz 베이스 모델 기준으로 정정]
  틀(작업대 긴 방향) = 로봇 ±X,  짧은 방향 = 로봇 ±Y
  베이스의 커넥터(버튼처럼 튀어나온 부분)는 짧은 방향(좁은 7cm 쪽)을 향해요.
  틀이 +X 쪽인지 -X 쪽인지는 아래 MOLD_SIGN 하나로 정해요.
  → 실물에서 티치펜던트 +X 조그를 눌러 틀 쪽으로 가면 +1, 반대로 가면 -1.
  물체들이 좌우(짧은 방향) 반대쪽에 있으면 MOLD_SHORT 등의 부호를 뒤집으세요.
"""
import math

MOLD_SIGN = -1.0         # ★ 틀(작업대 긴 쪽)이 로봇 +X면 +1, -X면 -1 (조그로 확인!)
SHORT_SIGN = +1.0        # ★ 짧은 방향 부호. 실물 베이스가 180° 돌아 설치돼 둘 다 -1
#   (베이스 180° 회전 = MOLD_SIGN과 SHORT_SIGN을 함께 뒤집기)


def pour_az():
    """주전자를 기울일 때 그리퍼 축이 넘어가는 방향(틀 반대쪽) — 그래야 주둥이가 틀 쪽으로 내려감"""
    return 0.0 if MOLD_SIGN < 0 else 180.0


def top_pose(x, y, z_tcp):
    """위에서 수직으로 내려 잡는 자세. z_tcp = 손가락 끝(잡는 점) 높이.
    A-C 차이가 손목 회전(KETTLE_YAW)이라, 기울일 때 B만 바꾸면 매끄럽게 기울어요."""
    a = pour_az()
    return [x, y, z_tcp + TOOL_LENGTH, a, 180.0, a - KETTLE_YAW]


def side_grip(long_, short, z):
    """원기둥(소스통) 옆면을 잡는 자세: 수평으로 다가가 손가락이 좌우로 닫힘"""
    q = pose_to(long_, short, z)
    q[5] = 0.0      # ★ 실물에서 손가락이 위아래로 닫히면 90.0
    return q


def radial_grip(long_, short, z, depth=0.0):
    """세워진 원기둥(소스통)을 옆면에서 잡는 자세: 1축을 돌려 통을 바라보고 수평 접근, 손가락 좌우로 닫힘.
    로봇 쪽에서 바깥을 바라보며 잡아요 (팔을 덜 뻗어도 돼서 E0509로도 닿음).
    depth = 손끝을 잡는 점보다 더 밀어 넣는 거리 → 손가락 끝이 아니라 안쪽 패드로 감싸 잡음"""
    x, y = MOLD_SIGN * long_, SHORT_SIGN * short
    r = math.hypot(x, y)
    ux, uy = x / r, y / r
    a = math.degrees(math.atan2(uy, ux))
    reach = TOOL_LENGTH - depth          # 플랜지 → 잡는 점
    return [x - ux * reach, y - uy * reach, z, a, 90.0, 0.0]


def grip_point(pose, depth=0.0):
    """잡기 자세(플랜지) → 실제로 물체를 잡는 점 (손끝에서 depth만큼 안쪽)"""
    zx, zy, zz = tool_dir(pose)
    d = TOOL_LENGTH - depth
    return pose[0] + d * zx, pose[1] + d * zy, pose[2] + d * zz


def outer_grip(long_, short, z):
    """세워진 통을 '바깥에서' 잡는 자세: 그리퍼가 통 너머에서 로봇 쪽을 바라보고
    손가락이 좌우로 닫힘. 로봇 가까이 둔 물체도 팔이 펴진 채 잡을 수 있어요."""
    x, y = MOLD_SIGN * long_, SHORT_SIGN * short
    r = math.hypot(x, y)
    ux, uy = x / r, y / r
    return [x + ux * TOOL_LENGTH, y + uy * TOOL_LENGTH, z, math.degrees(math.atan2(-uy, -ux)), 90.0, 0.0]


def pose_to(long_, short, z):
    """작업대 기준(틀 쪽 거리, 짧은 방향, 높이) → 로봇 posx [x,y,z,A,B,C].
    그리퍼가 틀 쪽을 수평으로 향하고 손가락이 위아래로 닫히는 자세예요.
    MOLD_SIGN을 바꾸면 모든 좌표와 자세가 자동으로 반대쪽으로 뒤집혀요."""
    a = 0.0 if MOLD_SIGN > 0 else 180.0         # 툴 z가 틀 쪽(±X)을 향하도록
    return [MOLD_SIGN * long_, SHORT_SIGN * short, z, a, 90.0, 90.0]

# ============================================================
# 1. 작업대 (도면 치수에서 계산)  — 긴 방향 = Y, 짧은 방향 = X
# ============================================================
TABLE_FWD = 901.0        # ★ 베이스 중심 → 틀 쪽 끝 (도면: 8.5 + 81.6cm. 가로 합계가 120과 6cm 차이 → 실측 확인)
TABLE_BACK = 238.0       # ★ 베이스 중심 → 반대쪽 끝 (15.3 + 8.5cm)
# 도면 세로: 7 + 21 + 26.5 = 54.5 ≈ 55cm
TABLE_SHORT_A = 375.0    # ★ +Y(넓은 쪽, 도면 아래) 끝: 26.5 + 10.5cm
TABLE_SHORT_B = 175.0    # ★ -Y(커넥터가 향하는 좁은 쪽) 끝: 7 + 10.5cm
PEDESTAL_LONG = 170.0    # ★ 로봇 받침대: 작업대 긴 방향 17cm
PEDESTAL_SHORT = 210.0   # ★ 받침대: 짧은 방향 21cm
PEDESTAL_H = 70.0        # ★ 받침대 높이 7cm (로봇 베이스가 이 위에 올라감)
HANDLE_APPROACH_BACK = 50.0  # ★ 틀 손잡이는 받침대와 가까워서 짧게 뒤에서 접근
TABLE_TOP_Z = -PEDESTAL_H   # 작업대 윗면 = 베이스보다 받침대 높이만큼 아래
# ---- 로봇 뒤쪽에 붙이는 보조 테이블 (소스통 올려두는 곳) ----
TABLE2_FWD = (-730.0, -238.0)    # ★ 틀 방향 범위 (로봇 뒤쪽, 기존 작업대 뒤 끝에 붙음) — 실측으로 교체
TABLE2_SHORT = (-245.0, 390.0)   # ★ 짧은 방향 범위 — 실측으로 교체

# ============================================================
# 2. 안전 영역  ← "틀 쪽으로 너무 가면 멈추는 문제" 방지
# ============================================================
SAFE_FWD_MAX = 600.0     # ★ 틀 방향 한계 (손끝 기준). 실제 멈춘 위치보다 10cm 안쪽으로
SAFE_RADIUS_MAX = 700.0  # ★ 베이스에서 수평 거리 한계
SAFE_RADIUS_MIN = 150.0  # ★ 베이스에 너무 붙는 자세 방지
SAFE_EDGE_MARGIN = 30.0  # ★ 작업대 가장자리 여유
TRANSIT_FREE_Z = TABLE_TOP_Z + 200.0  # ★ 이 높이 이상으로 들면 작업대 가장자리 검사를 안 함
#   (들거나 이동할 때는 테이블 밖으로 나가도 됨. 낮게 움직일 때만 가장자리 검사)
SAFE_Z_MIN = TABLE_TOP_Z + 15.0
SAFE_Z_MAX = 700.0

# ============================================================
# 3. 물체 길이 (실측하면 여기만 수정)
# ============================================================
TOOL_LENGTH = 160.0      # ★ 플랜지 면 → 손가락 끝 (브래킷 포함, 실측)
FINGER_LEN = 60.0        # ★ 손가락 길이 (손바닥 면 → 손끝, 실측으로 교체!) — 깊게 잡는 깊이가 이 값으로 정해져요
# ---- 깊게 잡기: 물체를 손바닥 바로 앞까지 밀어 넣어 손가락 전체로 감싸 잡음 ----
#   깊이 = 손가락 길이 - 물체 반두께 - PALM_GAP  (물체별 깊이는 각 물체 치수 아래에서 자동 계산)
PALM_GAP = 5.0           # ★ 손바닥 ↔ 물체 틈 (5mm). 실물에서 손바닥에 부딪히면 키우기
GRIPPER_YAW_DEG = 0.0    # ★ 그리퍼가 플랜지에 돌아가 붙은 각도 (RViz용)
GRIPPER_MOUNT_Z = 0.0    # ★ 브래킷 두께 (RViz용)

MOLD_L = 90.0            # ★ 틀 하나의 폭 9cm (짧은 방향). 틀 2개가 나란히 → 총 18cm
GRILL_FWD, GRILL_SHORT = 230.0, 180.0   # ★ 두 틀을 고정한 철판(석쇠) 23 x 18cm
MOLD_W = 77.5            # ★ 판 하나 깊이 = 펼친 길이 15.5cm의 절반
MOLD_FLOAT = 90.0        # ★ 틀이 작업대에서 떠 있는 높이
FISH_L, FISH_W = 80.0, 50.0
FISH_INSET = 10.0        # ★ 생선 칸 네모에서 가로·세로 1cm 안쪽에 타원
FISH_DEPTH = 10.0        # ★ 타원 파인 깊이 1cm
HANDLE_L, HANDLE_W, HANDLE_T = 150.0, 50.0, 20.0   # ★ 손잡이
STRING_LEN = 60.0        # ★ 위쪽 손잡이 → 끈 고리까지
SAUCE_DIA = 63.0         # ★ 소스통 지름 6.3cm
SAUCE_H = 225.0          # ★ 소스통 높이 22.5cm (노즐 포함)
# 주전자 손잡이는 뚜껑 위로 솟은 형태(하늘을 향함) → 그리퍼가 위에서 아래로 잡아요
KETTLE_SPOUT_DROP = 110.0   # ★ 손잡이 잡는 점 → 주둥이 끝까지 높이 차 (실측)
KETTLE_SPOUT_REACH = 110.0  # ★ 손잡이 잡는 점 → 주둥이 끝까지 수평 거리 (실측)
KETTLE_YAW = 90.0           # ★ 손가락이 손잡이를 가로질러 닫히도록 돌리는 각도 (0/90 중 실물에 맞게)
#   주둥이를 틀 위에 두면 그리퍼는 이만큼 뒤에 있게 돼요 → 팔이 덜 뻗어도 됨
NOZZLE_DROP = 0.0        # ★ 소스통: 그리퍼 TCP → 입구 끝 높이 차 (실측)

# ============================================================
# 4. 붓기 · 짜기
# ============================================================
POUR_GAP = 30.0          # ★ 주전자 주둥이 ↔ 틀 윗면 간격 (3cm)
FILL_GAP = 20.0          # ★ 소스통 입구 ↔ 틀 간격 (2cm)
MOLD_TOP_Z = TABLE_TOP_Z + MOLD_FLOAT + 20.0   # ★ 틀 윗면 높이 (틀 두께 2cm 가정)

# 생선 2칸 중심 오프셋 [틀방향, 짧은방향]  ★ 실물 보며 보정
# 도면 기준: 두 칸은 '짧은 방향(X)'으로 나란히 놓임 (손잡이 축과 수직)
FISH_OFFSETS = [[0.0, 0.0]]   # 판마다 생선 칸 1개(8x5cm), 판 가운데
POUR_LINE = 45.0         # ★ 붓는 동안 이동 길이 (머리→꼬리 = 손잡이 축 방향)
POUR_SPEED = 15.0        # ★ 붓는 이동 속도 (느릴수록 두껍게)
FILL_POINT_D = 0.0       # ★ 속재료 짜는 점 (생선 중심 기준, 틀방향 +)

# ============================================================
# 5. 티칭 좌표  ※ 전부 임시값! 실물에서 티칭해서 교체
#    pose_to(틀쪽 거리, 짧은방향, 높이) → MOLD_SIGN 따라 자동 변환
# ============================================================
# 틀 구조: 판 2장이 가운데 경첩으로 연결된 조개식. 기본(원래) 상태 = 활짝 열려 두 판이
# 틀 방향(Y)으로 나란히 펼쳐짐. 가까운 판 손잡이는 로봇 쪽, 먼 판 손잡이는 반대쪽.
# 각 판에는 생선 칸이 짧은 방향(X)으로 2개씩 있음.
MOLD_FWD = 170.0              # ★ 가까운 손잡이 '가로 중앙' 잡는 점까지 틀쪽 거리 (티칭으로 교체)
MOLD_SHORT = 120.0            # ★ 손잡이의 짧은방향(X) 위치
HANDLE_STICK = 40.0           # ★ 손잡이 잡는 점 → 판 가장자리까지 (손잡이 절반)
MOLD_GRIP = pose_to(MOLD_FWD, MOLD_SHORT, TABLE_TOP_Z + MOLD_FLOAT)
# 틀 1·2의 가운데 줄 (손잡이는 철판 가운데 1개, 두 틀이 그 양옆) — 1번 = 커넥터 쪽
MOLD_SHORTS = [MOLD_SHORT - MOLD_L / 2, MOLD_SHORT + MOLD_L / 2]
MOLD_GRIPS = [pose_to(MOLD_FWD, s, TABLE_TOP_Z + MOLD_FLOAT) for s in MOLD_SHORTS]
STRING_GRIP = pose_to(MOLD_FWD, MOLD_SHORT, TABLE_TOP_Z + MOLD_FLOAT - 60.0)  # (예비)
SAUCE_GRIP_DEPTH = FINGER_LEN - SAUCE_DIA / 2 - PALM_GAP   # 손끝이 통 중심을 지나 들어가는 거리 (= 23.5mm)
# 소스통 중심 (틀쪽 거리, 짧은방향) — 로봇 뒤 보조 테이블
SAUCE_SPOTS = {"팥": (-500.0, 180.0),      # ★ 팥
               "슈크림": (-420.0, 320.0)}  # ★ 슈크림 (팥과 16cm 간격)
# 로봇 쪽에서 잡기 (예전 outer_grip = 통 너머에서 잡기는 손목이 베이스에서 약 81cm →
#   E0509 팔 길이(약 78cm) 밖이라 "도달 실패"가 났어요)
SAUCE_PICK_REDBEAN = radial_grip(*SAUCE_SPOTS["팥"], TABLE_TOP_Z + SAUCE_H / 2, depth=SAUCE_GRIP_DEPTH)
SAUCE_PICK_CREAM = radial_grip(*SAUCE_SPOTS["슈크림"], TABLE_TOP_Z + SAUCE_H / 2, depth=SAUCE_GRIP_DEPTH)
REACH_TOL = 5.0          # ★ 목표 자세와 실제 도착 위치 차이 허용치 (mm)
OBJ_TRANSIT_Z = TABLE_TOP_Z + SAUCE_H + 140.0   # ★ 물체를 들고/잡으러 오갈 때 손 높이
#   (통을 든 채로도 통 바닥이 주전자·틀·다른 통 위로 지나가는 높이)
SAUCE_PICKS = {"팥": SAUCE_PICK_REDBEAN, "슈크림": SAUCE_PICK_CREAM}  # 주문 이름 → 통 위치
SAUCE_PICK = SAUCE_PICK_REDBEAN     # (이름이 안 맞을 때 쓰는 예비값)
KETTLE_FWD, KETTLE_SHORT = -650.0, 20.0     # ★ 비커 몸통 중심 — 로봇 뒤 보조 테이블, 9cm 상자 위 (손잡이가 로봇 쪽이라 멀리 둠)
KETTLE_HANDLE_Z = TABLE_TOP_Z + 140.0   # ★ 비커 손잡이 잡는 높이 = 상자 9cm + 손잡이 5cm
# ---- 비커(계량컵) 16.5 x 7.5 x 8.4cm: 손잡이는 옆면 ----
CUP_D, CUP_H = 75.0, 84.0       # ★ 비커 몸통 지름·높이
CUP_HANDLE_REACH = 82.5         # ★ 손잡이 잡는 점 → 몸통 중심 (수평)
CUP_GRIP_H = 50.0               # ★ 손잡이 잡는 높이 (작업대에서)
CUP_APPROACH = 40.0             # ★ 손잡이에 옆에서 다가가는 거리
POUR_GAP_CUP = CUP_H + 25.0     # ★ 몸통 바닥이 틀에 안 닿게 주둥이를 띄우는 높이 (바닥 ↔ 틀 2.5cm, 예전 1cm는 겹쳐 보였음)
CUP_HANDLE_T = 12.0             # ★ 비커 손잡이 굵기 (실측)
# 손바닥 앞 5mm까지 넣되, 손끝이 몸통 벽에 5mm 이상 떨어지게
CUP_GRIP_DEPTH = min(FINGER_LEN - CUP_HANDLE_T / 2 - PALM_GAP,
                     CUP_HANDLE_REACH - CUP_D / 2 - 5.0)
KETTLE_SPOUT_DROP = -(CUP_H - CUP_GRIP_H)                 # 잡는 점 → 주둥이 끝 높이 차
KETTLE_SPOUT_REACH = CUP_HANDLE_REACH - CUP_GRIP_DEPTH + CUP_D / 2 + 12.0  # 손끝 → 주둥이 끝 수평 거리 (깊게 잡은 만큼 짧아짐)
KETTLE_FWD, KETTLE_SHORT = -650.0, 20.0     # ★ 비커 몸통 중심 — 로봇 뒤 보조 테이블, 9cm 상자 위 (손잡이가 로봇 쪽이라 멀리 둠)
KETTLE_HANDLE_Z = TABLE_TOP_Z + 140.0   # ★ 비커 손잡이 잡는 높이 = 상자 9cm + 손잡이 5cm
CUP_BASE_H = 90.0                       # ★ 비커 밑 상자 높이 (9cm)
CUP_BOTTOM_Z = TABLE_TOP_Z + CUP_BASE_H


def _cup_pick():
    """비커: 손잡이가 로봇 쪽, 주둥이가 바깥쪽(로봇 반대).
    그리퍼가 로봇 쪽에서 바깥을 바라보며 손잡이를 옆으로 잡음.
    (예전 '비커 너머에서 로봇 쪽을 바라보는' 자세는 손목(5축)이 180° 가까이 꺾여야 해서
     E0509로는 NOT REACHABLE → 비커 앞에서 멈췄음)"""
    cx, cy = MOLD_SIGN * KETTLE_FWD, SHORT_SIGN * KETTLE_SHORT
    r = math.hypot(cx, cy)
    ux, uy = cx / r, cy / r
    gx, gy = cx - ux * CUP_HANDLE_REACH, cy - uy * CUP_HANDLE_REACH
    return radial_grip(gx * MOLD_SIGN, gy * SHORT_SIGN, KETTLE_HANDLE_Z, depth=CUP_GRIP_DEPTH)


KETTLE_PICK = _cup_pick()


APPROACH_BACK = 80.0     # ★ 잡기 전 뒤(베이스 쪽)에서 대기하는 거리
LIFT_HEIGHT = 250.0      # ★ 뒤집기 전 드는 높이 (+25cm)
FLIP_ANGLE = 180.0       # ★ 뒤집는 각도 (J6 = 손잡이 반대축 = 짧은 방향 축, 손잡이 끝이 중심)
RETREAT_UP = 100.0       # ★ 놓은 뒤 위로 빠지는 높이
RETREAT_SIDE = 50.0      # ★ 뒤집은 뒤 손잡이에서 옆(짧은 방향 Y)으로 떨어지는 거리 (5cm)
FLIP_BACK = True         # ★ 대기 후 다시 들어 J6 180° 되돌려 내려놓기. 끄면 뚜껑을 못 엶
#   (뒤집으면 틀이 손잡이 너머로 넘어가 경첩이 약 75cm → 열 때 손잡이가 약 90cm, E0509 약 78cm)

# ---- 제스처 (인사·대답) ----
BOW_J = [0.0, 30.0, 60.0, 0.0, 90.0, 0.0]   # ★ 인사 자세 (posj): 어깨를 30° 숙임, 손끝은 작업대 위 약 26cm
#   (예전 [0,40,100,0,40,0]은 E0509 모델로 계산하면 손끝 z=-128 → 작업대(-70)를 5.8cm 뚫었음)
GESTURE_AMP = 60.0       # ★ 도리도리 폭 (mm)
NOD_DOWN, NOD_UP = 25.0, 40.0   # ★ 끄덕: 아래로 2.5cm, 위로 4cm (예전 6cm 아래로 → 바닥 뚫음)
GESTURE_TIP_MIN_Z = TABLE_TOP_Z + 200.0   # ★ 인사·끄덕 중 손끝이 이 높이 밑으로 안 내려감 (작업대 위 20cm)
J1_LIMIT = 355.0
GESTURE_REPEAT = 2       # ★ 흔드는 횟수
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # ★ 시작/끝 자세 (posj)

# ============================================================
# 5-1. 공정 동작 값
# ============================================================
POUR_TILT_DEG = 25.0     # ★ 주전자 기울이는 각도 (25도)
POUR_HOLD1 = 2.0         # ★ 1차 반죽: 기울인 채 유지 (2초)
POUR_HOLD2 = 1.0         # ★ 2차 반죽: 유지 (1초)
POUR_RATIO = 2.0 / 3.0   # ★ 붓는 점: 틀 깊이의 2/3 지점 (손잡이에서 먼 쪽)
POUR_SIDE_TOL = 90.0     # ★ 틀끼리 그리퍼 방향(팔 기준) 차이 허용. 넘으면 비커 든 채 J4가 반 바퀴 감김
POUR_HOP = 80.0         # ★ 칸 사이 이동할 때 띄우는 높이
POUR_PLATE = 1           # ★ 반죽·소스를 넣는 판: 1 = 먼 판(마주보는 틀), 0 = 가까운 판
#   닫을 때 가까운 판(손잡이)이 경첩을 넘어와 먼 판 위를 덮어요 → 반죽은 먼 판에


def plate_k():
    """손잡이 잡는 점(손끝) → 반죽 넣는 판 가운데까지 틀방향 거리"""
    return HANDLE_STICK + MOLD_W * (0.5 + POUR_PLATE)

SAUCE_LIFT = 150.0       # ★ 소스통 수직으로 드는 높이
FLIP_CLEAR = 150.0       # ★ 입구 뒤집을 때 여유 높이 (통 절반 길이보다 크게)
SAUCE_GRIP_TO_MOUTH = 112.5  # ★ 잡는 점(통 중간) → 노즐 끝 (통 높이 절반, 실측으로 보정)
DISPENSE_INSET = 10.0    # ★ 틀 가장자리에서 1cm 안쪽까지만 짜기
DISPENSE_DIR = +1.0      # ★ '아래서 위로'의 시작 쪽: +1 = +Y(넓은 쪽)에서 시작. 반대면 -1
DISPENSE_HOP = 50.0      # ★ 짠 뒤 띄우는 높이

# ---- 뚜껑 (경첩 축 = 짧은 방향 X, 뚜껑은 틀 방향으로 열림) ----
# 경첩 = 두 판 사이 가운데. 위치는 손잡이에서 자동 계산:
#   경첩 = 잡는 점 + 그리퍼 길이 + 손잡이 절반 + 판 깊이
HINGE_FWD = MOLD_FWD + TOOL_LENGTH + HANDLE_STICK + MOLD_W   # (= 손잡이 티칭을 따라감)
HINGE_Z = TABLE_TOP_Z + MOLD_FLOAT + 20.0   # ★ 경첩 높이
LID_R = HANDLE_STICK + MOLD_W   # 경첩 → 반대쪽 손잡이 잡는 점 (판 깊이 + 손잡이 절반)
#   이 값이면 닫았을 때 위 손잡이가 아래 손잡이 위에 정확히 포개져요 (총 두께 4cm)
LID_OPEN_SIGN = +1.0     # 먼 쪽으로 열림 (조개식 구조상 고정)
# ⚠ 꼭 확인: 활짝 연 먼 손잡이 위치(HINGE_FWD + LID_R)가 틀 방향 한계 SAFE_FWD_MAX(600)
#   안이어야 해요. 지금 임시값: 경첩 460 + 반지름 130 = 590 < 600 ✓ (MOLD_FWD를 키우면 초과 주의)

# ---- 뚜껑 닫기: 손잡이를 '옆에서' 잡고 손목(J6)을 굴리며 닫기 ----
# (예전 방식은 뚜껑 너머에서 로봇 쪽을 보고 잡아야 해서 손목이 베이스에서 약 85cm → E0509로 안 닿음.
#  옆에서 잡으면 그리퍼 축 = 경첩 축과 나란해서, 뚜껑이 도는 만큼 J6만 돌면 돼요)
LID_GRAB_OUT = 65.0      # ★ 판 가장자리 → 손잡이 잡는 점 (그리퍼 몸통이 판에 안 닿게 손잡이 바깥쪽을 잡음)
LID_GRAB_R = MOLD_W + LID_GRAB_OUT   # 경첩 → 잡는 점 반지름
LID_SIDE_SIGN = -1.0     # ★ 그리퍼가 손잡이의 어느 옆에서 오는지: -1 = 로봇 중심선 쪽(짧은방향 작은 쪽)
LID_GRIP_DEPTH = FINGER_LEN - HANDLE_W / 2 - PALM_GAP   # 손잡이 폭 너머로 손끝을 넣는 깊이 (손바닥 틈 5mm)
LID_RELEASE_DEG = 15.0   # ★ 이 각도(거의 닫힘)에서 놓기 — 더 내리면 아래 손가락이 아래 손잡이에 걸려요
LID_APPROACH_BACK = LID_GRIP_DEPTH + HANDLE_W / 2 + 15.0   # 옆에서 다가갈 때 손끝이 손잡이 옆면에서 1.5cm 떨어진 곳부터
FINGER_T = 10.0          # ★ 손가락 두께 (실측!) — 뚜껑 열 때 아래 손가락이 위·아래 손잡이 사이 틈에 들어가야 해요
# 닫힌 상태: 아래 손잡이 윗면 / 위 손잡이 중심 높이 (lid_side_grip(0)과 같은 기하)
LOWER_HANDLE_TOP = TABLE_TOP_Z + MOLD_FLOAT + HANDLE_T / 2
UPPER_HANDLE_Z_CLOSED = 2 * HINGE_Z - (TABLE_TOP_Z + MOLD_FLOAT)
# 닫힌 뚜껑 손잡이에 들어갈 때 벌리는 폭: 아래 손가락이 아래 손잡이에 2mm 이상 안 닿을 만큼만
LID_CLOSED_OPEN_WIDTH = min(50.0, 2 * (UPPER_HANDLE_Z_CLOSED - (LOWER_HANDLE_TOP + FINGER_T + 2.0)))


def lid_side_grip(phi_deg, mold_short):
    """뚜껑 각도 phi(180 = 활짝 열림, 0 = 닫힘)에서 위 판 손잡이를 옆에서 잡은 플랜지 자세.
    손가락은 손잡이 윗면·아랫면을 집고(두께 방향), 뚜껑이 돌면 그만큼 C(손목)가 같이 돌아요."""
    ph = math.radians(phi_deg)
    h = (TABLE_TOP_Z + MOLD_FLOAT) - HINGE_Z   # 손잡이 중심이 판 윗면보다 낮은 거리 (열렸을 때 기준, 음수)
    fwd = HINGE_FWD - math.cos(ph) * LID_GRAB_R - math.sin(ph) * h
    z = HINGE_Z + math.sin(ph) * LID_GRAB_R - math.cos(ph) * h
    gx, gy = MOLD_SIGN * fwd, SHORT_SIGN * mold_short
    dy = -LID_SIDE_SIGN * SHORT_SIGN           # 툴이 향하는 방향(로봇 y): 플랜지 → 손잡이
    k = 1.0 if dy > 0 else -1.0                # A = +90° 또는 -90°
    reach = TOOL_LENGTH - LID_GRIP_DEPTH
    c = math.degrees(math.atan2(-math.cos(ph), k * MOLD_SIGN * math.sin(ph)))  # 손가락 = 손잡이 면에 수직
    return [gx, gy - dy * reach, z, 90.0 * k, 90.0, c]


# ---- 틀 뒤집기: 철판 옆 회전 손잡이 (회전축 = 짧은 방향, 로봇 중심선 쪽으로 튀어나옴) ----
# 철판은 회전축(축받이)에 고정돼 있고, 손잡이는 그 축과 같은 선 위에 있어야 해요.
# 그리퍼가 손잡이를 축 방향으로 물고 J6만 180° 돌리면 철판이 제자리에서 뒤집혀요 (들어 올리지 않음).
SIDE_HANDLE_FWD = HINGE_FWD   # ★ 임시값! 회전축(손잡이) 틀방향 거리 — 철판 중심. 실측으로 교체
SIDE_HANDLE_Z = HINGE_Z       # ★ 임시값! 회전축 높이 — 닫힌 틀 두께 중앙. 실측으로 교체
SIDE_HANDLE_T = 20.0          # ★ 손잡이 굵기 2cm
SIDE_HANDLE_LEN = 150.0       # ★ 철판 옆면에서 튀어나온 길이 15cm
SIDE_HANDLE_BASE = MOLD_SHORT - GRILL_SHORT / 2   # 철판의 로봇 중심선 쪽 옆면 (짧은방향)
SIDE_HANDLE_END = SIDE_HANDLE_BASE - SIDE_HANDLE_LEN  # 손잡이 끝 (짧은방향)


def side_handle_grip():
    """옆 손잡이를 축 방향으로 무는 자세. 손잡이 끝이 손바닥 앞 PALM_GAP까지 들어오게 깊게 잡고,
    손가락은 수평(틀방향)으로 닫혀서 위에서 내려와 손잡이를 사이에 끼울 수 있어요."""
    tip_s = SIDE_HANDLE_END + PALM_GAP + FINGER_LEN    # 손끝 위치 (짧은방향)
    k = 1.0 if SHORT_SIGN > 0 else -1.0                # 툴이 +짧은방향(철판 쪽)을 향함
    return [MOLD_SIGN * SIDE_HANDLE_FWD, SHORT_SIGN * (tip_s - TOOL_LENGTH), SIDE_HANDLE_Z,
            90.0 * k, 90.0, 0.0]


# ---- 꼬챙이로 다 익은 붕어빵 꺼내기 (꼬챙이 1개 = 붕어빵 1개, 반죽 부은 순서대로) ----
SKEWER_ROD = 150.0            # ★ 쇠 막대 길이 (손잡이 제외) 15cm
SKEWER_HANDLE_D = 40.0        # ★ 손잡이 지름 4cm
SKEWER_HANDLE_L = 70.0        # ★ 손잡이 길이(높이) 7cm
SKEWER_ROD_D = 4.0            # ★ 막대 굵기 (실측)
SKEWER_TILT_DEG = 6.0         # ★ 붕어빵 가운데 가로선에서 5~7° 사선 (손잡이 쪽이 높고 끝이 살짝 내려가며 찌름)
SKEWER_YAW_DEG = 10.0         # ★ 위에서 볼 때 손잡이 쪽을 로봇 쪽으로 살짝 비틀기 (옆 회전 손잡이와 안 부딪히게)
SKEWER_AIM_Z = MOLD_TOP_Z     # ★ 꿰는 선이 지나는 붕어빵 가운데 높이 (아래 칸 1cm + 위 칸 1cm의 중간 = 판 윗면)
SKEWER_START_GAP = 10.0       # ★ 꿰기 시작할 때 끝이 붕어빵에서 떨어진 거리
SKEWER_TIP_INSET = 10.0       # ★ 다 꿴 뒤 끝이 붕어빵 반대쪽 끝보다 안쪽에 멈추는 거리 (뚫고 나가지 않게)
SKEWER_SPEED = 15.0           # ★ 찌르는 속도 (mm/s)
SKEWER_LIFT = 100.0           # ★ 꿴 뒤 수직으로 드는 높이
SKEWER_GRIP_DEPTH = FINGER_LEN - SKEWER_HANDLE_D / 2 - PALM_GAP   # 손끝이 손잡이 중심을 지나는 깊이 (손바닥 틈 5mm)
# 보관: 눕혀서 꽂아 둠. 막대 끝이 -짧은방향(꽂이 안)을 향하고 손잡이가 +짧은방향으로 나와 있음
# 꼬챙이는 1개뿐: 붕어빵을 컵에 넣은 뒤 꼬챙이만 빼서 다음 붕어빵에 다시 써요. 한 판이 끝나면 꽂이에 돌려놓기
SKEWER_SLOT = (-270.0, 0.0)   # ★ 임시값! (틀쪽 거리, 손잡이 중심 짧은방향) — 보조 테이블
SKEWER_PULL_SPEED = 20.0      # ★ 컵에 넣은 붕어빵에서 꼬챙이를 위로 뽑는 속도 (느릴수록 붕어빵이 덜 따라 올라옴)
SKEWER_REST_Z = TABLE_TOP_Z + 50.0   # ★ 꽂혀 있을 때 꼬챙이 축 높이
SKEWER_IN_HOLDER = 60.0       # ★ 막대가 꽂이에 들어가 있는 길이 → 이만큼 + 2cm 뽑아요

# ---- 종이컵: 손님이 들고 있는 컵에 꼬챙이를 세워(끝이 아래) 붕어빵을 넣고, 꼬챙이만 뽑기 ----
# 인사하는 방향(+X)에서 90° 돌린 곳에 손님이 컵을 들고 있어요. 붕어빵마다 새 컵 (손님이 바꿔 듦)
PAPER_CUP_D, PAPER_CUP_H = 72.0, 90.0   # ★ 종이컵 윗지름·높이 (실측)
CUP_AZ_DEG = 90.0             # ★ 컵 방향 (로봇 기준 각도). +90 = +Y(작업대 넓은 쪽), 반대쪽이면 -90
CUP_R = 450.0                 # ★ 베이스에서 컵 중심까지 수평 거리 45cm
CUP_TOP_Z = TABLE_TOP_Z + 350.0   # ★ 컵 윗면 높이 = 작업대 위 35cm
CUP_FISH_GAP = 10.0           # ★ 붕어빵 아래 끝 ↔ 컵 바닥 간격
CUP_ROTATE_CLEAR = 30.0       # ★ 컵 위에서 꼬챙이를 세울 때 붕어빵 끝이 컵 윗면보다 이만큼 높게
CUP_SWAP_SEC = 3.0            # ★ 두 번째 붕어빵부터 넣기 전에 손님이 새 컵으로 바꿀 시간 (초)
SKEWER_TRANSIT_Z = OBJ_TRANSIT_Z + 30.0   # 꼬챙이를 들고 옮길 때 플랜지 높이 (막대가 소스통 위로 지나감)
SKEWER_EDGE_MARGIN = 10.0     # ★ 꿰는 동안만 작업대 가장자리 여유를 3cm → 1cm로 (손잡이가 작업대 끝 근처까지 감)


def _zyz(R):
    """회전행렬 → 두산 posx 각도 (A, B, C) [deg] (ZYZ)"""
    b = math.atan2(math.hypot(R[0][2], R[1][2]), R[2][2])
    if abs(math.sin(b)) < 1e-6:                       # 툴이 정확히 위/아래 → A만으로 표현
        return math.degrees(math.atan2(-R[1][0], -R[0][0]) if R[2][2] < 0 else math.atan2(R[1][0], R[0][0])), \
            math.degrees(b), 0.0
    return (math.degrees(math.atan2(R[1][2], R[0][2])), math.degrees(b),
            math.degrees(math.atan2(R[2][1], -R[2][0])))


def skewer_pose(grip, d):
    """꼬챙이 손잡이를 위에서 가로로 무는 플랜지 자세.
    grip = 손잡이 잡는 점(로봇 xyz), d = 꼬챙이 끝이 향하는 단위벡터.
    툴 x = 꼬챙이 방향, 툴 z = 꼬챙이에 수직인 '아래', 손가락(툴 y)은 손잡이 양옆을 집음."""
    dn = (0.0, 0.0, -1.0)
    k = sum(dn[i] * d[i] for i in range(3))
    z = [dn[i] - k * d[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in z)); z = [v / n for v in z]
    y = [z[1] * d[2] - z[2] * d[1], z[2] * d[0] - z[0] * d[2], z[0] * d[1] - z[1] * d[0]]
    R = [[d[0], y[0], z[0]], [d[1], y[1], z[1]], [d[2], y[2], z[2]]]
    reach = TOOL_LENGTH - SKEWER_GRIP_DEPTH
    return [grip[i] - z[i] * reach for i in range(3)] + list(_zyz(R))


def skewer_rest_pose():
    """꽂이에 눕혀 꽂힌 꼬챙이의 손잡이 가운데를 무는 자세"""
    f, s = SKEWER_SLOT
    return skewer_pose((MOLD_SIGN * f, SHORT_SIGN * s, SKEWER_REST_Z), (0.0, -SHORT_SIGN, 0.0))


def skewer_dir(i):
    """i번 틀 붕어빵을 꿰는 방향 (끝이 향하는 단위벡터). 모든 틀을 로봇 쪽에서 +짧은방향으로 꿰요.
    (반죽 부은 순서대로 1번을 먼저 꺼내니까, 2번을 꿸 땐 1번 칸이 이미 비어 있음.
     반대쪽에서 꿰면 손잡이가 작업대 끝 밖으로 나가요)"""
    yw, tl = math.radians(SKEWER_YAW_DEG), math.radians(SKEWER_TILT_DEG)
    hx, hy = MOLD_SIGN * math.sin(yw), SHORT_SIGN * math.cos(yw)   # 끝 쪽이 틀방향(+)으로 살짝 → 손잡이는 로봇 쪽
    return (hx * math.cos(tl), hy * math.cos(tl), -math.sin(tl))


def frame_pose(grip, x, z, depth):
    """잡는 점 grip, 툴 x축·z축(단위벡터) → 플랜지 posx (손끝이 grip을 depth만큼 지나감)"""
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    R = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    reach = TOOL_LENGTH - depth
    return [grip[i] - z[i] * reach for i in range(3)] + list(_zyz(R))


def cup_center():
    """손님이 든 컵의 중심 (로봇 xy)"""
    t = math.radians(CUP_AZ_DEG)
    return CUP_R * math.cos(t), CUP_R * math.sin(t)


def _cup_frames(a_deg, n=6):
    """컵 위에서: 그리퍼를 수평(방향 a)으로 두고 꼬챙이를 옆으로 눕혀 든 자세 → J6만 굴려 꼬챙이를 세움(끝이 아래)
    n단계 + 컵에 넣은 자세. 손잡이가 툴 축 위에 있어서 세우는 동안 손잡이 위치가 그대로예요."""
    cx, cy = cup_center()
    back = SKEWER_ROD + SKEWER_HANDLE_L / 2
    a = (math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg)), 0.0)   # 그리퍼가 향하는 수평 방향
    h = (a[1], -a[0], 0.0)                     # 눕혔을 때 꼬챙이 방향 (a × h = 아래 가 되도록)
    fish_low = SKEWER_TIP_INSET                # 붕어빵이 끝보다 더 나와 있는 길이
    z_hi = CUP_TOP_Z + CUP_ROTATE_CLEAR + fish_low + back                    # 세울 때 손잡이 높이
    z_in = CUP_TOP_Z - PAPER_CUP_H + CUP_FISH_GAP + fish_low + back          # 컵에 넣었을 때 손잡이 높이
    rot = []
    for k in range(n + 1):
        t = math.radians(90.0 * k / n)
        x = [math.cos(t) * h[i] + math.sin(t) * (0.0, 0.0, -1.0)[i] for i in range(3)]
        rot.append(frame_pose((cx, cy, z_hi), x, a, SKEWER_GRIP_DEPTH))
    put = frame_pose((cx, cy, z_in), (0.0, 0.0, -1.0), a, SKEWER_GRIP_DEPTH)
    return rot, put


def _cup_dir():
    """컵에 넣을 때 그리퍼 방향: 모든 자세가 안전 검사를 통과하고 팔이 가장 덜 접히는 방향"""
    best = None
    for ad in range(0, 360, 15):
        rot, put = _cup_frames(ad)
        try:
            for q in rot + [put]:
                check_pose(q, "컵 계획")
            for p, q in zip(rot + [rot[-1]], rot[1:] + [put]):
                check_path(p, q, "컵 계획")
        except ValueError:
            continue
        sc = min(min(arm_fold(q) for q in rot + [put]), WRIST_REACH_MAX - max(arm_fold(q) for q in rot + [put]))
        if best is None or sc > best[0]:
            best = (sc, ad)
    if best is None:
        raise ValueError("[컵 계획 실패] 손님 컵 위치에 꼬챙이를 넣을 수 있는 그리퍼 방향이 없어요 "
                         "→ CUP_R / CUP_TOP_Z를 조정하세요")
    return best[1]


def cup_plan():
    """손님 컵: (세우기 경유 자세들 [0]=눕힘 … [-1]=세움, 넣기 자세)"""
    return _cup_frames(_cup_dir())


def skewer_pierce(i):
    """i번 틀 붕어빵 꿰기: (시작 자세, 끝 자세, 시작 끝점, 끝 끝점). 꿰는 선은 붕어빵 가운데를 지나요."""
    fwd = MOLD_FWD + TOOL_LENGTH + HANDLE_STICK + MOLD_W / 2     # 가까운 판(아래 판) 생선 칸 중심
    c = (MOLD_SIGN * fwd, SHORT_SIGN * MOLD_SHORTS[i], SKEWER_AIM_Z)
    d = skewer_dir(i)
    back = SKEWER_ROD + SKEWER_HANDLE_L / 2                       # 끝 → 손잡이 가운데
    out = []
    for t in (-(FISH_L / 2 + SKEWER_START_GAP), FISH_L / 2 - SKEWER_TIP_INSET):
        tip = [c[k] + d[k] * t for k in range(3)]
        out.append((skewer_pose([tip[k] - d[k] * back for k in range(3)], d), tip))
    return out[0][0], out[1][0], out[0][1], out[1][1]


LID_STEP_DEG = 15.0      # ★ 호 경유점 간격
STRING_SLACK = 20.0      # ★ 끈 당길 때 반지름 여유
STRING_RELEASE_DEG = 120.0  # ★ 끈을 이 각도까지만 당기고 놓기 (그 뒤는 중력이 뚜껑을 마저 넘김)
#   끈 호는 반지름이 커서 180°까지 따라가면 틀 방향 한계를 넘어요. 90~130° 사이에서 조절

# ============================================================
# 6. 속도 · 시간
# ============================================================
SPEED_SCALE_REAL = 0.2   # ★ 실물 속도 배율
VEL_J, ACC_J = 40.0, 40.0
VEL_L, ACC_L = 100.0, 100.0
VEL_SLOW_L = 30.0
VEL_FLIP = 30.0
J6_LIMIT = 355.0
J4_UNWIND_DEG = 180.0    # ★ 빈손 단계 시작 때 J4가 이보다 감겨 있으면 홈으로 가서 풀기 (한계 ±360°)
J_WOUND_WARN = 340.0     # 도달 실패 시 이 각도를 넘은 관절이 있으면 '관절 한계' 원인으로 안내
LID_J6_SWEEP = -240.0    # ★ 닫기 180° 동안 J6가 도는 양 (에뮬레이터 IK로 잰 값 약 -220°). 뒤집기·되돌리기는 0 쪽으로 풀었다 다시 감음
#   여유가 없으면 닫기 전에 빈손으로 공중에서 J6를 360° 미리 감아 둠
BAKE_SEC_SIDE1 = 10.0    # ★ 뚜껑 닫고 기다리는 시간 (10초, 실물은 굽는 시간으로)
BAKE_SEC_SIDE2 = 10.0    # ★ 뒤집은 채 잡고 기다리는 시간 (10초)


# ============================================================
# 좌표 변환 · 안전 검사 (수정하지 않아도 돼요)
# ============================================================
def fwd_of(x, y):
    """로봇 좌표 → 틀 방향 거리 (틀 방향 = X, MOLD_SIGN 반영)"""
    return x * MOLD_SIGN


def tool_dir(pose):
    """툴 z(그리퍼가 향하는 방향) 단위벡터"""
    a, b = math.radians(pose[3]), math.radians(pose[4])
    return (math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.cos(b))


def tool_tip(pose):
    zx, zy, zz = tool_dir(pose)
    return pose[0] + TOOL_LENGTH * zx, pose[1] + TOOL_LENGTH * zy, pose[2] + TOOL_LENGTH * zz


SHOULDER_Z = 155.5       # E0509 어깨(2축) 높이 (대략)
WRIST_BACK = 124.0       # 플랜지 → 손목 중심 (대략)
FOLD_MIN = 230.0         # ★ 어깨~손목 거리가 이보다 짧으면 3축이 한계(155°) 근처로 접힘


def arm_fold(pose):
    """어깨~손목 중심 거리(mm). 작을수록 팔이 많이 접혀 3축 한계에 걸리기 쉬움"""
    u = tool_dir(pose)
    w = [pose[i] - u[i] * WRIST_BACK for i in range(3)]
    return math.dist(w, [0.0, 0.0, SHOULDER_Z])


WRIST_REACH_MAX = 740.0  # ★ 어깨~손목 거리 최대 (E0509 409+367=776mm, 다 펴면 특이점이라 여유 둠)
FOLD_PATH_MIN = 240.0   # ★ 직선 이동 중 어깨~손목 최소 거리 (이보다 짧으면 3축 한계에 걸림)


def check_path(a, b, label="", n=30):
    """a → b 직선 이동 중간 지점들까지 팔 접힘 검사 (끝점만 괜찮아도 중간에 걸릴 수 있어요)"""
    worst = None
    for k in range(n + 1):
        p = [a[i] + (b[i] - a[i]) * k / n for i in range(3)] + list(b[3:6])
        f = arm_fold(p)
        if worst is None or f < worst[0]:
            worst = (f, p)
    if worst[0] < FOLD_PATH_MIN:
        w = [round(v) for v in worst[1][:3]]
        raise ValueError(f"[안전 정지] {label}: 이동 중 팔이 너무 접힘 (지점 {w}, "
                         f"어깨~손목 {worst[0]:.0f}mm < {FOLD_PATH_MIN:.0f}) → 물체를 로봇에서 더 멀리 두세요")


def check_pose(pose, label=""):
    """목표 자세가 안전 영역 밖이면 ValueError. 플랜지와 손끝 둘 다 검사."""
    problems = []
    m = SAFE_EDGE_MARGIN
    for name, (x, y, z) in (("플랜지", tuple(pose[:3])), ("손끝", tool_tip(pose))):
        fwd = fwd_of(x, y)
        r = math.hypot(x, y)
        if fwd > SAFE_FWD_MAX:
            problems.append(f"{name} 틀방향 {fwd:.0f} > 한계 {SAFE_FWD_MAX:.0f}")
        if r > SAFE_RADIUS_MAX:
            problems.append(f"{name} 거리 {r:.0f} > 최대 {SAFE_RADIUS_MAX:.0f}")
        if r < SAFE_RADIUS_MIN:
            problems.append(f"{name} 거리 {r:.0f} < 최소 {SAFE_RADIUS_MIN:.0f}")
        if z < TRANSIT_FREE_Z and name == "손끝":   # 손끝만 작업대 위면 됨 (팔·손목은 밖으로 나가도 OK), 높이 들면 면제
            s_ = y * SHORT_SIGN
            on1 = (-TABLE_BACK + m <= fwd <= TABLE_FWD - m) and (-TABLE_SHORT_B + m <= s_ <= TABLE_SHORT_A - m)
            on2 = (TABLE2_FWD[0] + m <= fwd <= TABLE2_FWD[1] + m) and (TABLE2_SHORT[0] + m <= s_ <= TABLE2_SHORT[1] - m)
            if not (on1 or on2):
                problems.append(f"{name} (틀방향 {fwd:.0f}, 짧은방향 {s_:.0f}) 작업대 밖 (낮은 높이)")
        if z < SAFE_EDGE_MARGIN and abs(x) < PEDESTAL_LONG / 2 + m and abs(y) < PEDESTAL_SHORT / 2 + m:
            problems.append(f"{name} 받침대와 충돌 (x={x:.0f}, y={y:.0f}, z={z:.0f})")
        if not (SAFE_Z_MIN <= z <= SAFE_Z_MAX):
            problems.append(f"{name} z={z:.0f} 높이 범위({SAFE_Z_MIN:.0f}~{SAFE_Z_MAX:.0f}) 밖")
    if arm_fold(pose) < FOLD_MIN:
        problems.append(f"팔이 너무 접힘 (어깨~손목 {arm_fold(pose):.0f}mm < {FOLD_MIN:.0f}, 3축 한계 위험)")
    if arm_fold(pose) > WRIST_REACH_MAX:
        problems.append(f"팔이 안 닿음 (어깨~손목 {arm_fold(pose):.0f}mm > {WRIST_REACH_MAX:.0f}, "
                        "E0509 최대 약 776mm) → 물체를 로봇 쪽으로 당기거나 잡는 방향을 바꾸세요")
    if problems:
        raise ValueError(f"[안전 정지] {label}: " + " / ".join(problems))


def _seg_rect_gap(px, py, rect):
    """점(로봇 xy) ↔ 사각형(틀쪽 거리, 짧은방향 범위) 거리"""
    f, s = px * MOLD_SIGN, py * SHORT_SIGN
    (f0, f1), (s0, s1) = rect
    dx = max(f0 - f, 0.0, f - f1)
    dy = max(s0 - s, 0.0, s - s1)
    return math.hypot(dx, dy)


def _obstacles():
    """붓는 동안 피해야 할 것: 틀 손잡이 2개(가까운/먼), 소스통 2개"""
    grip_f = MOLD_FWD + TOOL_LENGTH
    edge = grip_f + HANDLE_STICK
    far = edge + 2 * MOLD_W
    rects = []
    for ms in HANDLE_SHORTS:                        # 앞·뒤 손잡이
        hs = (ms - HANDLE_W / 2, ms + HANDLE_W / 2)
        rects += [((edge - HANDLE_L, edge), hs), ((far, far + HANDLE_L), hs)]
    for f, s in SAUCE_SPOTS.values():           # 소스통 중심 (틀쪽 거리, 짧은방향)
        r = SAUCE_DIA / 2
        rects.append(((f - r, f + r), (s - r, s + r)))
    return rects


POUR_CLEAR = 15.0        # ★ 비커·그리퍼와 손잡이·소스통 사이 최소 여유 (위에서 본 거리)
GRIPPER_HALF_W = 45.0    # ★ 그리퍼 몸통 반폭


def _pour_poses(S, A):
    """주둥이 목표 S, 그리퍼 방향 A → (수평 자세 F, 기울인 자세 T, 이동 높이 H)"""
    u = (math.cos(math.radians(A)), math.sin(math.radians(A)))
    L = KETTLE_SPOUT_REACH + TOOL_LENGTH
    D = KETTLE_SPOUT_DROP
    ph = math.radians(POUR_TILT_DEG)
    F = [S[0] - u[0] * L, S[1] - u[1] * L, S[2] + D, A, 90.0, 0.0]
    hor = -L * math.cos(ph) + D * math.sin(ph)
    vz = L * math.sin(ph) + D * math.cos(ph)
    T = [S[0] + u[0] * hor, S[1] + u[1] * hor, S[2] + vz, A, 90.0 + POUR_TILT_DEG, 0.0]
    H = list(F); H[2] = OBJ_TRANSIT_Z
    return F, T, H


def _pour_clear(S, A, rects):
    """위에서 봤을 때 비커 몸통·그리퍼 몸통이 손잡이·소스통과 안 겹치는지 (최소 거리 반환)"""
    u = (math.cos(math.radians(A)), math.sin(math.radians(A)))
    c = (S[0] - u[0] * (CUP_D / 2 + 12.0), S[1] - u[1] * (CUP_D / 2 + 12.0))
    gaps = [_seg_rect_gap(c[0], c[1], r) - CUP_D / 2 for r in rects]
    tcp = KETTLE_SPOUT_REACH
    for d in range(0, int(TOOL_LENGTH + WRIST_BACK) + 1, 20):
        p = (S[0] - u[0] * (tcp + d), S[1] - u[1] * (tcp + d))
        gaps += [_seg_rect_gap(p[0], p[1], r) - GRIPPER_HALF_W for r in rects]
    return min(gaps)


def pour_side(S, A):
    """그리퍼가 팔(베이스→손목 방향) 기준으로 몇 도 돌아가 있는지 (-180~180).
    부호가 바뀌면(팔 왼쪽 ↔ 오른쪽) 비커를 든 채 손목이 반 바퀴 넘어가요."""
    F = _pour_poses(S, A)[0]
    az = math.degrees(math.atan2(F[1], F[0]))
    return (A - az + 180.0) % 360.0 - 180.0


def plan_pour(grip, prev_side=None):
    """주둥이를 이 틀의 노란 생선 칸 안에 두고, 손잡이·소스통을 피하면서
    팔이 가장 편한 그리퍼 방향을 고름 → (주둥이 목표 S, 방향 A)
    prev_side = 앞 틀에서 쓴 pour_side. 주면 같은 쪽(차이 POUR_SIDE_TOL 이내)만 고름.
    (예전엔 1번 틀 -92°, 2번 틀 +88°로 반대쪽을 골라 → 비커 든 채 J4가 틀마다 반 바퀴씩
     같은 방향으로 감겨 2번 붓기 뒤 J4 362° 한계 초과 → 뚜껑 닫기에서 멈췄음)"""
    md, t0 = tool_dir(grip), tool_tip(grip)
    k = plate_k()
    cx, cy = t0[0] + md[0] * k, t0[1] + md[1] * k
    rects = _obstacles()
    best = None
    for df, ds in ((0, 0), (8, 0), (-8, 0), (0, 15), (0, -15)):   # 칸 안쪽에서만
        S = [cx + md[0] * df - md[1] * ds, cy + md[1] * df + md[0] * ds, MOLD_TOP_Z + POUR_GAP_CUP]
        for A in range(0, 360, 5):
            if _pour_clear(S, A, rects) < POUR_CLEAR:
                continue
            if prev_side is not None:
                d = (pour_side(S, A) - prev_side + 180.0) % 360.0 - 180.0
                if abs(d) > POUR_SIDE_TOL:
                    continue
            F, T, H = _pour_poses(S, A)
            try:
                for q in (F, T, H):
                    check_pose(q, "붓기 계획")
            except ValueError:
                continue
            r = math.hypot(F[0], F[1])
            outward = (F[0] * math.cos(math.radians(A)) + F[1] * math.sin(math.radians(A))) / r
            if outward < -0.6:          # 그리퍼가 로봇 쪽으로 완전히 되돌아보는 자세만 제외
                continue
            fold = min(arm_fold(F), arm_fold(T))
            score = -abs(fold - 450.0) + 30.0 * outward
            if best is None or score > best[0]:
                best = (score, S, float(A))
        if best:
            break
    if best is None:
        if prev_side is not None:
            raise ValueError("[붓기 계획 실패] 앞 틀과 같은 쪽으로 부을 방향이 없어요 "
                             "(반대쪽으로 가면 J4가 감겨 한계 초과) — 비커/틀 위치를 조정하세요")
        raise ValueError("[붓기 계획 실패] 손잡이·소스통을 피해서 부을 방향이 없어요 — 비커/틀 위치를 조정하세요")
    return best[1], best[2]


def plan_pours(molds):
    """틀 여러 개 붓기 계획: 첫 틀에서 고른 쪽을 나머지 틀도 따라감 → [(i, S, A)]"""
    plans, side = [], None
    for i, grip in molds:
        S, A = plan_pour(grip, side)
        side = pour_side(S, A)
        plans.append((i, S, A))
    return plans


# ================= 뚜껑 닫기·뒤집기·열기: 1·2번 틀 각자의 손잡이를 '옆에서' 잡기 =================
# 철판 가운데 손잡이는 없음. 틀마다 가까운 판·먼 판에 손잡이가 하나씩 (틀 가운데 줄).
# 1번 틀은 커넥터 쪽(좁은 쪽)에서, 2번 틀은 넓은 쪽에서 옆으로 들어와 잡아요 (옆 틀 손잡이와 안 부딪히게).
SIDE_GRIP_SIGNS = [-1.0, +1.0]
HANDLE_SHORTS = [MOLD_SHORT]     # ★ 손잡이 줄 (철판 가운데, 앞 손잡이·뒤 손잡이 한 쌍)   # ★ 틀별 그리퍼가 들어오는 쪽: -1 = 좁은 쪽, +1 = 넓은 쪽
LID_GRIP_R = 150.0       # ★ 경첩 → 손잡이 잡는 점 거리 (틀에서 떨어진 손잡이 바깥쪽을 잡음)
HANDLE_GAP_CLOSED = 10.0  # ★ 닫았을 때 위·아래 손잡이 사이 틈 (실측 약 1cm)
STACK_T = HANDLE_T + HANDLE_GAP_CLOSED   # 닫힌 상태: 아래 손잡이 중심 → 위 손잡이 중심 (겹친 전체 두께 = 2*HANDLE_T + 틈 = 5cm)


def _zyz_mat(a, b, c):
    a, b, c = (math.radians(t) for t in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc, sb * sc, cb]]


def _mat_zyz(R):
    b = math.atan2(math.hypot(R[0][2], R[1][2]), R[2][2])
    if abs(math.sin(b)) < 1e-6:
        a, c = 0.0, math.atan2(R[1][0], R[0][0]) if R[2][2] > 0 else math.atan2(R[1][0], -R[0][0])
    else:
        a = math.atan2(R[1][2], R[0][2])
        c = math.atan2(R[2][1], -R[2][0])
    return [math.degrees(a), math.degrees(b), math.degrees(c)]


def _axis_rot(k, deg):
    t = math.radians(deg); c, s_ = math.cos(t), math.sin(t); x, y, z = k
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s_, x * z * (1 - c) + y * s_],
            [y * x * (1 - c) + z * s_, c + y * y * (1 - c), y * z * (1 - c) - x * s_],
            [z * x * (1 - c) - y * s_, z * y * (1 - c) + x * s_, c + z * z * (1 - c)]]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _side_base(mi):
    vy = -SIDE_GRIP_SIGNS[mi] * SHORT_SIGN
    return _zyz_mat(90.0 if vy > 0 else -90.0, 90.0, 90.0), (0.0, vy, 0.0)


def side_lid_pose(theta, z_extra=0.0, mi=0):
    """mi번 틀: 경첩 각 theta(0 = 먼 판 펼쳐짐, 180 = 가까운 판 위에 덮임)일 때 먼 판 손잡이를 옆에서 잡은 자세"""
    R0, v = _side_base(mi)
    hx = MOLD_SIGN * HINGE_FWD
    hy = SHORT_SIGN * HANDLE_SHORTS[mi]
    hz = MOLD_GRIP[2]
    th = math.radians(theta)
    tx = hx + MOLD_SIGN * LID_GRIP_R * math.cos(th)
    tz = hz + LID_GRIP_R * math.sin(th) + z_extra
    R = _mm(_axis_rot((0.0, -MOLD_SIGN, 0.0), theta), R0)
    return [tx - v[0] * TOOL_LENGTH, hy - v[1] * TOOL_LENGTH, tz - v[2] * TOOL_LENGTH, *_mat_zyz(R)]


def side_flip_pose(beta, lift=0.0, mi=0):
    """mi번 틀: 닫힌 뒤 겹친 손잡이를 옆에서 잡고, 손잡이 축(틀 방향)으로 beta도 돌린 자세"""
    R0, v = _side_base(mi)
    tx = MOLD_SIGN * (HINGE_FWD - LID_GRIP_R)
    ty = SHORT_SIGN * HANDLE_SHORTS[mi]
    tz = MOLD_GRIP[2] + STACK_T / 2 + lift
    k = (-1.0 if v[1] > 0 else 1.0, 0.0, 0.0)          # 돌리는 동안 그리퍼 몸통이 '위'로 지나감
    R = _mm(_axis_rot(k, beta), R0)
    u = [R[i][2] for i in range(3)]
    return [tx - u[0] * TOOL_LENGTH, ty - u[1] * TOOL_LENGTH, tz - u[2] * TOOL_LENGTH, *_mat_zyz(R)]


# ---- 뒤집기: 틀 몸통 가운데를 축으로 180° → 몸통은 받침대 위 제자리, 손잡이만 반대쪽(로봇 쪽)으로 ----
# 닫힌 틀 몸통 = 먼 판 자리(경첩 ~ 경첩 + 판 깊이). 손잡이(J6 축)를 중심으로 돌리면 몸통이 손잡이 너머
# (약 67~75cm)로 넘어가 받침대 밖에 놓였음 → 돌린 뒤 손을 수평으로 FLIP_SHIFT 만큼 옮겨서 몸통을 제자리로.
FLIP_PIVOT_FWD = HINGE_FWD + MOLD_W / 2          # ★ 몸통 가운데 (틀 방향). 실물 받침대 중심으로 맞추기
FLIP_SHIFT = 2.0 * (FLIP_PIVOT_FWD - (HINGE_FWD + LID_GRIP_R))   # 손잡이 → 반대쪽 손잡이 자리 (음수 = 로봇 쪽)


def flip_shift_vec():
    """뒤집은 뒤 손을 옮기는 방향·거리 (로봇 x, y)"""
    return (MOLD_SIGN * FLIP_SHIFT, 0.0)


# ---- 뚜껑 열기: 다 구운 뒤 위 판 '옆면'을 옆에서 잡고 경첩을 축으로 펼치기 ----
# 닫힌 손잡이 사이 틈(1cm)엔 손가락(끝 9mm, 15mm 지점 14mm)이 못 들어가서 위 손잡이만 잡을 수 없어요.
# 판 사이 틈은 4cm라 아래 손가락이 들어감 → 위 판 옆면 가장자리를 위·아래로 집어서 엶.
PLATE_GAP_CLOSED = 40.0       # ★ 닫았을 때 위·아래 판 사이 틈 (실측 4cm)
PLATE_T = 10.0                # ★ 위 판 옆 가장자리 두께 — 임시값! 실측해서 grip_params PLATE_* 폭도 같이 맞추기
PLATE_GRIP_IN = 15.0          # ★ 판 옆면에서 안쪽으로 손끝을 넣는 깊이 (생선 칸에 안 닿게)
PLATE_GRIP_FWD = HINGE_FWD + MOLD_W / 2   # ★ 잡는 점 틀 방향 위치 (닫힌 위 판 가운데)
PLATE_SIDE_SHORT = MOLD_SHORT - GRILL_SHORT / 2   # 판 옆면 (로봇 중심선 쪽, 짧은방향)
HINGE_SEAM_Z = MOLD_GRIP[2] + STACK_T / 2         # ★ 경첩 축 높이 = 위·아래 판 사이 틈 가운데
OPEN_RELEASE_DEG = 15.0       # ★ 다 펴기 이만큼 전에 놓기 — 더 펴면 위쪽 손가락이 판 밑(받침대)에 깔려요
OPEN_J6_SWEEP = -LID_J6_SWEEP * (180.0 - OPEN_RELEASE_DEG) / 180.0   # 열 때는 닫을 때 반대로 돎


def plate_open_pose(theta, mi=0):
    """위 판 옆면을 옆에서 잡은 자세. theta = 연 각도 (0 = 닫힘, 180 = 활짝).
    손가락은 판 윗면·아랫면을 집고(아래 손가락은 판 사이 틈), 판이 돌면 같이 돌아요."""
    R0, v = _side_base(mi)                         # 툴이 판 옆면 안쪽(짧은방향)을 봄, 손가락 위아래
    p0 = [MOLD_SIGN * PLATE_GRIP_FWD,
          SHORT_SIGN * PLATE_SIDE_SHORT + v[1] * PLATE_GRIP_IN,
          HINGE_SEAM_Z + PLATE_GAP_CLOSED / 2 + PLATE_T / 2]
    c = [MOLD_SIGN * HINGE_FWD, p0[1], HINGE_SEAM_Z]
    Rh = _axis_rot((0.0, -MOLD_SIGN, 0.0), theta)  # +theta = 경첩 너머 로봇 쪽으로 펼침
    d = [p0[k] - c[k] for k in range(3)]
    p = [c[i] + sum(Rh[i][k] * d[k] for k in range(3)) for i in range(3)]
    return pose_with(p, _mm(Rh, R0))


def stack_rot(mi=0):
    """닫힌 자리(겹친 손잡이)를 옆에서 잡을 때 그리퍼 회전 (뒤집기 전)"""
    return _side_base(mi)[0]


def stack_tcp(lift=0.0, mi=0):
    """닫힌 뒤 겹친 손잡이 2장(두께 합 4cm)의 가운데 = 잡는 점. lift = 들어 올린 높이"""
    return side_tcp(0.0, STACK_T / 2 + lift, mi)


# ---- 제자리에서 다시 잡기: 닫은 자리에서 그리퍼만 벌렸다 조여 겹친 손잡이를 잡고, 그대로 뒤집고, 그대로 펼침 ----
def side_tcp(theta, z_extra=0.0, mi=0):
    """경첩 각 theta일 때 그 판 손잡이 잡는 점 (0 = 먼 쪽, 180 = 가까운 쪽)"""
    th = math.radians(theta)
    return [MOLD_SIGN * HINGE_FWD + MOLD_SIGN * LID_GRIP_R * math.cos(th),
            SHORT_SIGN * HANDLE_SHORTS[mi],
            MOLD_GRIP[2] + LID_GRIP_R * math.sin(th) + z_extra]


def pose_with(tcp, R):
    """손끝 위치 + 회전행렬 → 로봇 자세"""
    u = [R[i][2] for i in range(3)]
    return [tcp[k] - u[k] * TOOL_LENGTH for k in range(3)] + _mat_zyz(R)


def hinge_rot(theta, R_at_far):
    """먼 쪽(0°)에서의 그리퍼 회전 R_at_far 를 경첩 축으로 theta 만큼 돌린 회전"""
    return _mm(_axis_rot((0.0, -MOLD_SIGN, 0.0), theta), R_at_far)


def handle_rot(beta, R_start, mi=0):
    """손잡이 축(틀 방향)으로 beta 만큼 돌린 회전. 돌리는 동안 그리퍼 몸통이 위로 지나가는 방향"""
    vy = R_start[1][2]
    return _mm(_axis_rot((-1.0 if vy > 0 else 1.0, 0.0, 0.0), beta), R_start)


# ================= 돌림판: 경첩 봉을 잡고 틀을 수평으로 90° 돌린 뒤, 겹친 손잡이를 끝에서 잡아 뒤집기 =================
ROD_R = 110.0            # ★ 돌림판 중심 → 경첩 봉 끝 잡는 점 (철판 반폭 9cm + 2cm)
ROD_SIDE = -1.0          # ★ 잡는 봉 끝: -1 = 좁은 쪽(커넥터 쪽), +1 = 넓은 쪽
ROD_DIA = 10.0           # ★ 봉 지름 (그림용)


def _turn_center():
    return (MOLD_SIGN * HINGE_FWD, SHORT_SIGN * MOLD_SHORT)


def turn_deg():
    """먼 쪽에 겹친 손잡이가 봉이 있던 쪽(좁은 쪽)으로 오도록 돌리는 각도"""
    a = math.atan2(0.0, MOLD_SIGN)
    b = math.atan2(ROD_SIDE * SHORT_SIGN, 0.0)
    d = math.degrees(b - a)
    return (d + 180.0) % 360.0 - 180.0


def _rot2(v, deg):
    t = math.radians(deg); c, s_ = math.cos(t), math.sin(t)
    return (v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c)


def _end_on(tcp, inward):
    """끝에서(축 방향으로) 잡는 자세: 그리퍼가 inward 방향(수평)을 보고 손가락은 위아래로 닫힘"""
    a = math.degrees(math.atan2(inward[1], inward[0]))
    return pose_with(tcp, _zyz_mat(a, 90.0, 90.0))


def rod_pose(t):
    """돌림판을 t도 돌렸을 때 경첩 봉 끝을 '위에서' 잡은 자세 (손가락이 봉을 가로질러 닫힘)"""
    cx, cy = _turn_center()
    d = _rot2((0.0, ROD_SIDE * SHORT_SIGN), t)
    tcp = [cx + d[0] * ROD_R, cy + d[1] * ROD_R, HINGE_Z]
    yaw = math.degrees(math.atan2(d[1], d[0]))
    return pose_with(tcp, _zyz_mat(0.0, 180.0, -yaw))


def turned_stack_pose(lift=0.0, z_extra=0.0):
    """돌린 뒤: 겹친 손잡이를 '끝에서' 잡은 자세"""
    cx, cy = _turn_center()
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())
    tcp = [cx + d[0] * LID_GRIP_R, cy + d[1] * LID_GRIP_R, MOLD_GRIP[2] + STACK_T / 2 + lift + z_extra]
    return _end_on(tcp, (-d[0], -d[1]))


def turned_open_tcp(theta, z_extra=0.0):
    """돌린 뒤 펼치기: 위 손잡이 잡는 점이 경첩을 돌아 반대편으로 (0 → 180)"""
    cx, cy = _turn_center()
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())
    th = math.radians(theta)
    return [cx + d[0] * LID_GRIP_R * math.cos(th), cy + d[1] * LID_GRIP_R * math.cos(th),
            MOLD_GRIP[2] + LID_GRIP_R * math.sin(th) + z_extra]


def turned_hinge_rot(theta, R_start):
    """돌린 뒤의 경첩 축으로 theta 돌린 회전 — 손잡이가 먼저 위로 들리는 방향"""
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())
    k = (d[1], -d[0], 0.0)
    return _mm(_axis_rot(k, theta), R_start)


def pour_z():
    """주전자로 부을 때 플랜지 목표 z (위에서 잡은 자세, 주둥이 끝이 틀 위 3cm)"""
    return MOLD_TOP_Z + POUR_GAP_CUP + KETTLE_SPOUT_DROP     # 옆잡기(수평)라 플랜지 높이 = 잡는 점 높이
def fill_z():
    """소스통으로 짤 때 플랜지 목표 z (입구 끝이 틀 위 3cm)"""
    return MOLD_TOP_Z + FILL_GAP + NOZZLE_DROP
