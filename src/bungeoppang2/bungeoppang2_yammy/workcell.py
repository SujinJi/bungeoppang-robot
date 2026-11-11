"""(호환용) 예전 import를 그대로 쓰기 위한 파일.

값은 이제 두 파일로 나뉘었어요:
  - grip_params.py : ★ 그리퍼 힘·폭 (조이는 값)
  - positions.py   : ★ 위치·물체 길이·안전 영역·속도

기존 코드(flip_mold_node, gripper_ctrl, gripper_viz)는 `from bungeoppang2_yammy import workcell as W`를
그대로 써도 되고, 새 코드는 grip_params / positions를 직접 import 하세요.
"""
from bungeoppang2_yammy.grip_params import *   # noqa: F401,F403
from bungeoppang2_yammy.positions import *     # noqa: F401,F403

# 예전 이름 ↔ 새 이름 연결 (기존 코드 호환)
from bungeoppang2_yammy.grip_params import HANDLE_OPEN_WIDTH as GRIP_OPEN_WIDTH        # noqa: F401
from bungeoppang2_yammy.grip_params import HANDLE_GRIP_WIDTH as MOLD_GRIP_WIDTH        # noqa: F401
from bungeoppang2_yammy.grip_params import HANDLE_GRIP_CURRENT as MOLD_GRIP_CURRENT    # noqa: F401
