#!/usr/bin/env python3
"""상황판 노드: 손님 · 로봇 질문 · 들은 말 · 인식 결과(팥=빨강, 슈크림=노랑 칸) · 현재 단계를
한글 이미지로 그려 /bungeoppang/hud 로 보냄 (RViz2 Image 패널로 확인)"""
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PImage, ImageDraw, ImageFont

W, H = 760, 440
RED, YELLOW = (140, 30, 20), (255, 215, 75)


def korean_font(size):
    try:
        path = subprocess.run(["fc-match", "-f", "%{file}", ":lang=ko"],
                              capture_output=True, text=True).stdout.strip()
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()


def cut(s, n=30):
    return s if len(s) <= n else s[:n - 1] + "…"


class Hud(Node):
    def __init__(self):
        super().__init__("bungeoppang_hud")
        self.customer, self.customer_t = "손님 대기 중", 0.0
        self.robot_say = "-"
        self.listening, self.listen_t = False, 0.0
        self.heard = "-"
        self.counts = None          # {"팥": n, "슈크림": m} 또는 None
        self.failed = False
        self.step = "-"
        self.f_title, self.f_text, self.f_chip = korean_font(30), korean_font(24), korean_font(22)
        self.create_subscription(String, "/bungeoppang/customer", self.on_customer, 10)
        self.create_subscription(String, "/bungeoppang/listen", self.on_listen, 10)
        self.create_subscription(String, "/bungeoppang/order", self.on_order, 10)
        self.create_subscription(String, "/bungeoppang/step", self.on_step, 10)
        self.pub = self.create_publisher(Image, "/bungeoppang/hud", 5)
        self.create_timer(0.5, self.render)

    def on_customer(self, msg):
        self.customer = "손님 도착!" if msg.data == "arrived" else "손님이 떠남"
        self.customer_t = time.time()

    def on_listen(self, msg):
        self.robot_say, self.listening, self.listen_t = msg.data, True, time.time()
        self.heard, self.counts, self.failed = "-", None, False

    def on_order(self, msg):
        self.listening = False
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.heard = d.get("text", "") or "(아무 말도 못 들음)"
        if "counts" in d:
            self.counts, self.failed = d["counts"], False
        else:
            self.counts, self.failed = None, True

    def on_step(self, msg):
        self.step = msg.data.strip()

    def chip(self, dr, x, y, label, n, color):
        dr.rounded_rectangle((x, y, x + 150, y + 40), radius=8, fill=color)
        dark = color == YELLOW
        dr.text((x + 12, y + 7), f"{label} ×{n}", font=self.f_chip, fill=(40, 30, 0) if dark else (255, 255, 255))

    def render(self):
        img = PImage.new("RGB", (W, H), (28, 30, 36))
        dr = ImageDraw.Draw(img)
        dr.text((20, 14), "붕어빵 로봇 상황판", font=self.f_title, fill=(255, 210, 120))
        arrived = "도착" in self.customer and time.time() - self.customer_t < 60
        y = 70

        def row(label, draw):
            nonlocal y
            dr.rectangle((20, y, 100, y + 44), fill=(55, 60, 72))
            dr.text((32, y + 8), label, font=self.f_text, fill=(255, 255, 255))
            draw(118, y)
            y += 62

        row("손님", lambda x, yy: dr.text((x, yy + 8), self.customer, font=self.f_text,
                                         fill=(120, 230, 120) if arrived else (200, 200, 200)))
        row("로봇", lambda x, yy: dr.text((x, yy + 8), cut(self.robot_say), font=self.f_text, fill=(220, 220, 255)))
        heard = f"듣는 중... {time.time() - self.listen_t:.0f}초" if self.listening else f"들은 말: {cut(self.heard, 26)}"
        row("음성", lambda x, yy: dr.text((x, yy + 8), heard, font=self.f_text,
                                         fill=(255, 230, 120) if self.listening else (230, 230, 230)))

        def draw_result(x, yy):
            if self.counts:
                cx = x
                if self.counts.get("팥", 0):
                    self.chip(dr, cx, yy + 2, "팥", self.counts["팥"], RED)
                    cx += 165
                if self.counts.get("슈크림", 0):
                    self.chip(dr, cx, yy + 2, "슈크림", self.counts["슈크림"], YELLOW)
                    cx += 165
                total = sum(self.counts.values())
                dr.text((cx + 6, yy + 8), f"= 총 {total}개", font=self.f_text, fill=(230, 230, 230))
            elif self.failed:
                dr.text((x, yy + 8), "못 알아들음 → 다시 질문", font=self.f_text, fill=(255, 120, 120))
            else:
                dr.text((x, yy + 8), "-", font=self.f_text, fill=(200, 200, 200))
        row("인식", draw_result)

        sc = RED if "[팥]" in self.step else YELLOW if "[슈크림]" in self.step else (180, 220, 255)
        row("단계", lambda x, yy: dr.text((x, yy + 8), cut(self.step), font=self.f_text,
                                         fill=(255, 140, 120) if sc == RED else sc))
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "hud"
        msg.height, msg.width = H, W
        msg.encoding = "rgb8"
        msg.step = W * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Hud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
