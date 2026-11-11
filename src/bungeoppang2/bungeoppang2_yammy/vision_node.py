"""사람 인식 노드 (웹캠 + YOLO)

카메라에 사람이 보이면:
  - /bungeoppang/person_count  (std_msgs/Int32)   현재 보이는 사람 수
  - /bungeoppang/person        (std_msgs/Bool)    없음→있음으로 바뀌는 순간 True 한 번
    (손님이 '새로 왔을 때'만 인사하도록 orchestrator가 이 신호를 써요)

설치 (한 번만):
  pip install ultralytics opencv-python --break-system-packages
  (첫 실행 때 yolov8n.pt 모델을 자동 다운로드)

실행:  ros2 run bungeoppang2 vision_node
       ros2 run bungeoppang2 vision_node --show   ← 화면에 인식 결과 창 띄우기
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

# ================= ★ 조절하는 값 =================
CAMERA_INDEX = 0        # ★ 웹캠 번호 (노트북 내장 0, USB 캠은 1, 2...)
CONF_THRESH = 0.5       # ★ 사람 판정 확신도 (오인식 많으면 올리기)
MIN_BOX_RATIO = 0.05    # ★ 화면 높이 대비 사람 크기 최소 비율 (멀리 지나가는 사람 무시)
DETECT_HZ = 5.0         # ★ 초당 인식 횟수 (올리면 반응 빠름, GPU 사용량 증가)
ARRIVE_FRAMES = 3       # ★ 연속 n번 보여야 '손님 도착' (깜빡임 방지)
LEAVE_SEC = 3.0         # ★ 이만큼 안 보이면 '손님 떠남'
# =================================================


class VisionNode(Node):
    def __init__(self, show=False):
        super().__init__("vision_node")
        self.count_pub = self.create_publisher(Int32, "/bungeoppang/person_count", 10)
        self.arrive_pub = self.create_publisher(Bool, "/bungeoppang/person", 10)
        self.show = show

        self.get_logger().info("YOLO 모델 로딩 중...")
        from ultralytics import YOLO
        self.model = YOLO("yolov8n.pt")       # ★ 더 정확히: yolov8s.pt
        import cv2
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {CAMERA_INDEX}번을 열 수 없어요 (CAMERA_INDEX 확인)")
        self.get_logger().info("카메라 준비 완료. 손님 기다리는 중...")

        self.present = False
        self.seen_streak = 0
        self.last_seen = 0.0
        self.create_timer(1.0 / DETECT_HZ, self.loop)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        h = frame.shape[0]
        results = self.model(frame, verbose=False, classes=[0], conf=CONF_THRESH)  # class 0 = person
        boxes = [b for b in results[0].boxes if (b.xyxy[0][3] - b.xyxy[0][1]) / h > MIN_BOX_RATIO]
        n = len(boxes)
        self.count_pub.publish(Int32(data=n))

        now = time.time()
        if n > 0:
            self.last_seen = now
            self.seen_streak += 1
            if not self.present and self.seen_streak >= ARRIVE_FRAMES:
                self.present = True
                self.get_logger().info(f"👋 손님 도착 ({n}명)")
                self.arrive_pub.publish(Bool(data=True))
        else:
            self.seen_streak = 0
            if self.present and now - self.last_seen > LEAVE_SEC:
                self.present = False
                self.get_logger().info("손님 떠남")
                self.arrive_pub.publish(Bool(data=False))

        if self.show:
            self.cv2.imshow("bungeoppang vision", results[0].plot())
            self.cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode(show="--show" in sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
