#!/usr/bin/env python3
"""손님 감지 노드: 웹캠에서 얼굴이 일정 시간 보이면 /bungeoppang/customer 로 "arrived" 를 보냄
  ros2 run bungeoppang customer                          # 웹캠 (기본)
  ros2 run bungeoppang customer --ros-args -p mode:=key   # Enter = 손님 도착 (카메라 없이 테스트)
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Customer(Node):
    def __init__(self):
        super().__init__("bungeoppang_customer")
        self.mode = self.declare_parameter("mode", "camera").value
        self.cam = int(self.declare_parameter("camera", 0).value)
        self.hold_s = float(self.declare_parameter("hold_s", 1.0).value)
        self.leave_s = float(self.declare_parameter("leave_s", 3.0).value)
        self.show = bool(self.declare_parameter("show", True).value)
        self.pub = self.create_publisher(String, "/bungeoppang/customer", 10)

    def send(self, what):
        self.pub.publish(String(data=what))
        self.get_logger().info("손님 도착!" if what == "arrived" else "손님이 떠남")

    def run_key(self):
        while rclpy.ok():
            input("\n[Enter] = 손님 도착 ▶ ")
            self.send("arrived")

    def run_camera(self):
        import cv2
        import mediapipe as mp
        cap = cv2.VideoCapture(self.cam)
        if not cap.isOpened():
            self.get_logger().error("카메라를 열 수 없어요. mode:=key 로 실행해보세요.")
            return
        det = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.6)
        seen_since, gone_since, here = None, None, False
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            res = det.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            face = bool(res.detections)
            now = time.time()
            if face:
                gone_since = None
                seen_since = seen_since or now
                if not here and now - seen_since >= self.hold_s:
                    here = True
                    self.send("arrived")
            else:
                seen_since = None
                if here:
                    gone_since = gone_since or now
                    if now - gone_since >= self.leave_s:
                        here = False
                        self.send("left")
            if self.show:
                if res.detections:
                    h, w = frame.shape[:2]
                    for d in res.detections:
                        b = d.location_data.relative_bounding_box
                        cv2.rectangle(frame, (int(b.xmin * w), int(b.ymin * h)),
                                      (int((b.xmin + b.width) * w), int((b.ymin + b.height) * h)), (0, 200, 0), 2)
                cv2.putText(frame, "CUSTOMER" if here else "waiting", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0) if here else (0, 0, 255), 2)
                cv2.imshow("bungeoppang customer", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        cap.release()


def main():
    rclpy.init()
    node = Customer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run_key() if node.mode == "key" else node.run_camera()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
