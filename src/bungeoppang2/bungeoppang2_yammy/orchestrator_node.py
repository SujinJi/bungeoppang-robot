"""총괄 노드 (상태 머신)

  대기 ──손님 인식(/bungeoppang/person)──▶ 주문 받기 ──음성 주문(/bungeoppang/order)──▶ 큐에 추가
                                               │ (제한시간 안에 주문 없으면 대기로)
  큐에 주문이 있고 로봇이 쉬는 중이면 ──▶ /bungeoppang/start_bake 발행 → bake_sequence가 굽기 시작
  bake_sequence가 끝나면 /bungeoppang/bake_done 발행 → 다음 주문

실행:  ros2 run bungeoppang2 orchestrator
수동 주문(음성 없이 테스트):
  ros2 topic pub --once /bungeoppang/order std_msgs/String "{data: '팥x1'}"
"""
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ================= ★ 조절하는 값 =================
AUTO_GREET = True      # ★ 대기 중 말을 걸면 손님으로 보고 인사 (카메라 연결 후 False 가능)
ORDER_TIMEOUT = 30.0   # ★ 손님 인식 후 주문을 기다리는 시간 (초)
GREETING = "어서오세요! 무엇으로 주문하시겠어요? 팥 또는 슈크림, 몇 개인지 말씀해 주세요."   # ★ 인사말
MAX_QUEUE = 4          # ★ 대기 주문 최대 수
# =================================================


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.state = "IDLE"            # IDLE / LISTEN / BAKING
        self.queue = deque()
        self.timer = None

        self.create_subscription(Bool, "/bungeoppang/person", self.on_person, 10)
        self.create_subscription(String, "/bungeoppang/order", self.on_order, 10)
        self.create_subscription(Bool, "/bungeoppang/bake_done", self.on_done, 10)
        self.say_pub = self.create_publisher(String, "/bungeoppang/say", 10)
        self.bake_pub = self.create_publisher(String, "/bungeoppang/start_bake", 10)
        self.gesture_pub = self.create_publisher(String, "/bungeoppang/gesture", 10)
        self.listen_pub = self.create_publisher(Bool, "/bungeoppang/listening", 10)
        self.create_subscription(String, "/bungeoppang/voice_text", self.on_text, 10)
        self.no_timer = None
        self.create_timer(0.5, self.tick)
        self.get_logger().info("총괄 노드 시작 (상태: IDLE)")

    def say(self, text):
        """안내 멘트. TTS를 붙이면 /bungeoppang/say를 구독해서 읽게 하면 돼요."""
        self.get_logger().info(f"💬 {text}")
        self.say_pub.publish(String(data=text))

    # 상태: IDLE(손님 기다림) → LISTEN(주문 듣는 중) → ORDERED(주문 확정, 더 안 들음) → BAKING
    # ---- 손님 인식 ----
    def on_person(self, msg):
        if msg.data and self.state == "IDLE":
            self.state = "LISTEN"
            self.gesture_pub.publish(String(data="greet"))   # ㄱ자 인사
            self.say(GREETING)
            self.say("🎤 지금 말씀해 주세요")
            self._restart_timer()

    def _restart_timer(self):
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(ORDER_TIMEOUT, self.listen_timeout)

    def _stop_timers(self):
        for t in (self.timer, self.no_timer):
            if t:
                t.cancel()
        self.timer = None
        self.no_timer = None

    def listen_timeout(self):
        if self.state == "LISTEN":
            self.say("주문이 없어서 대기할게요.")
            self.state = "IDLE"
        self._stop_timers()

    # ---- 말소리: 주문으로 못 알아들으면 '다시 말씀해주세요' ----
    def on_text(self, msg):
        if self.state == "IDLE" and AUTO_GREET:
            self.on_person(Bool(data=True))           # 말을 걸면 손님이 온 것으로 보고 인사부터
            return                                    # (이 말에 주문이 들어 있으면 바로 접수됨)
        if self.state != "LISTEN":
            return                                    # 주문 받는 중이 아니면 무시
        if self.no_timer:
            self.no_timer.cancel()
        self.no_timer = self.create_timer(1.5, self.say_no)   # 1.5초 안에 주문이 안 오면 못 알아들은 것

    def say_no(self):
        if self.no_timer:
            self.no_timer.cancel()
            self.no_timer = None
        if self.state == "LISTEN":
            self.gesture_pub.publish(String(data="no"))     # 좌우 도리도리
            self.say("다시 말씀해주세요.")
            self._restart_timer()                           # 다시 30초 기다림

    # ---- 주문: 한 손님에 한 번만 받고, 받으면 더 이상 듣지 않음 ----
    def on_order(self, msg):
        if self.state != "LISTEN":
            self.get_logger().info(f"(주문 안 받는 중 — 상태 {self.state}) 무시: {msg.data}")
            return
        total = []
        for part in msg.data.split(","):                 # "팥x2,슈크림x1" / "팥x1"
            filling, _, count = part.strip().partition("x")
            try:
                n = int(count or 1)
            except ValueError:
                continue
            total += [filling.strip()] * n
        total = [t for t in total if t][:MAX_QUEUE]
        if not total:
            return
        self._stop_timers()
        self.state = "ORDERED"                           # ← 여기서 듣기 끝
        self.served = 0                                  # 이 손님 붕어빵 번호 (꼬챙이·종이컵 번호)
        self.queue.extend(total)
        self.gesture_pub.publish(String(data="yes"))     # 위아래 끄덕
        said = ", ".join(f"{k} {total.count(k)}개" for k in dict.fromkeys(total))
        self.say(f"{said} 주문 받았어요! 바로 구워 드릴게요.")

    # ---- 굽기 (틀 2개라 한 번에 2개씩) ----
    def tick(self):
        self.listen_pub.publish(Bool(data=self.state in ("IDLE", "LISTEN")))   # 굽는 중엔 마이크 끔
        if self.state == "ORDERED" and self.queue:
            batch = [self.queue.popleft()]
            if self.queue:
                batch.append(self.queue.popleft())
            self.state = "BAKING"
            start = getattr(self, "served", 0)
            self.served = start + len(batch)
            cmd = ",".join(batch)                        # 예: "팥,슈크림"
            if self.queue:
                self.say(f"굽기 시작: {cmd} (남은 {len(self.queue)}개는 이 판을 꺼낸 뒤 처음부터 다시 구워요)")
            else:
                self.say(f"굽기 시작: {cmd}")
            self.bake_pub.publish(String(data=f"{cmd}@{start}"))   # @번호 = 꼬챙이·종이컵을 몇 번부터 쓸지

    def on_done(self, msg):
        if msg.data:
            self.say("다 구워졌어요! 맛있게 드세요 🐟")
        else:
            self.say("문제가 생겨서 멈췄어요. 터미널의 [안전 정지] 줄을 확인해 주세요.")
            self.queue.clear()
        if self.queue:
            self.state = "ORDERED"                       # 남은 주문(3개 이상)이면 이어서 굽기
        else:
            self.state = "IDLE"
            self.say("다음 손님을 기다릴게요.")


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
