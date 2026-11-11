"""RH-P12-RN 그리퍼를 DRL로 제어 (두산 컨트롤러 DrlStart 서비스 사용)
폭(m)을 받아 위치값으로 바꾼 뒤, DRL 코드를 컨트롤러에 보내고 끝날 때까지 기다려요."""
import time
from dsr_msgs2.srv import DrlStart, GetDrlState

STROKE = 0.109        # 최대 벌림 (m)
POS_OPEN = 0          # 완전히 열렸을 때 위치값
POS_CLOSE = 740       # ★ 완전히 닫혔을 때 위치값 (실제 그리퍼로 확인 후 수정)

# ▼▼▼ 사용하는 RH-P12-RN용 DRL 코드를 여기에 붙여 넣으세요 ▼▼▼
# {pos} 자리에 위치값(정수), {current} 자리에 힘 제한이 들어가요. ({current}는 선택)
DRL_TEMPLATE = """
# 예시 자리 - 실제 그리퍼 DRL 코드로 교체
tp_log("gripper -> {pos}")
"""
# ▲▲▲ 여기까지 ▲▲▲


class DrlGripper:
    def __init__(self, node, ns="/dsr01", robot_system=0):
        self.node = node
        self.robot_system = robot_system
        self.start_cli = node.create_client(DrlStart, f"{ns}/drl/drl_start")
        self.state_cli = node.create_client(GetDrlState, f"{ns}/drl/get_drl_state")

    def _call(self, cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"서비스 없음: {cli.srv_name}")
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"서비스 응답 없음: {cli.srv_name}")
        return fut.result()

    def width_to_pos(self, width):
        w = min(max(width, 0.0), STROKE)
        return int(round(POS_OPEN + (POS_CLOSE - POS_OPEN) * (1.0 - w / STROKE)))

    def run(self, code, timeout=10.0):
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = code
        if not self._call(self.start_cli, req).success:
            raise RuntimeError("DRL 실행 실패")
        time.sleep(0.2)
        t0 = time.time()
        while time.time() - t0 < timeout:          # 끝날 때까지 대기 (1 = STOP)
            st = self._call(self.state_cli, GetDrlState.Request())
            if st.success and st.drl_state == 1:
                return
            time.sleep(0.1)
        raise RuntimeError("그리퍼 DRL이 시간 안에 끝나지 않음")

    def move(self, width):
        pos = self.width_to_pos(width)
        self.run(DRL_TEMPLATE.format(pos=pos))
        return pos
