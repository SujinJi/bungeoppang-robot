"""그리퍼 제어 래퍼.

- real 모드: 기존 gripper_drl.py의 DRL 코드(DRL_TEMPLATE)를 컨트롤러로 보내 실제 그리퍼를 움직여요.
- virtual 모드: 실제 그리퍼가 없으니 DRL은 건너뛰고 RViz 표시만 해요.
- 두 모드 모두 /bungeoppang/gripper_width (mm) 토픽을 내보내서 RViz 그리퍼 손가락이 따라 움직여요.
"""
import time
from std_msgs.msg import Float64

from bungeoppang2_yammy import workcell as W


class GripperCtrl:
    def __init__(self, node, real=False):
        self.node = node
        self.real = real
        self.pub = node.create_publisher(Float64, "/bungeoppang/gripper_width", 10)
        self.drl = None
        if real:
            # 기존 파일 재사용: DRL_TEMPLATE에 붙여 넣은 실제 그리퍼 코드를 그대로 씀
            from bungeoppang2_yammy.gripper_drl import DrlGripper, DRL_TEMPLATE, STROKE
            self.drl = DrlGripper(node, robot_system=0)
            self.template = DRL_TEMPLATE
            self.stroke_mm = STROKE * 1000.0

    def _show(self, width_mm):
        msg = Float64()
        msg.data = float(width_mm)
        self.pub.publish(msg)

    def move(self, width_mm, current=None, wait=None):
        """width_mm: 손가락 사이 폭 (작을수록 세게 조임)
        current:  힘 제한 (DRL 코드에 {current} 자리가 있을 때만 적용)"""
        width_mm = max(0.0, min(109.0, width_mm))
        self.node.get_logger().info(f"그리퍼 → {width_mm:.0f}mm"
                                    + (f", current {current}" if current else ""))
        if self.real:
            pos = self.drl.width_to_pos(width_mm / 1000.0)
            # {pos}, {current} 자리를 채워서 실행 (템플릿에 없는 자리는 무시돼요)
            code = self.template.format(pos=pos, current=current or W.MOLD_GRIP_CURRENT)
            self.drl.run(code)
        self._show(width_mm)
        time.sleep(W.GRIP_WAIT_SEC if wait is None else wait)

    def open(self):
        self.move(W.GRIP_OPEN_WIDTH, wait=0.3)

    def grip_mold(self):
        # ★ 조이는 정도는 workcell.py의 MOLD_GRIP_WIDTH, MOLD_GRIP_CURRENT에서 수정
        self.move(W.MOLD_GRIP_WIDTH, current=W.MOLD_GRIP_CURRENT)
