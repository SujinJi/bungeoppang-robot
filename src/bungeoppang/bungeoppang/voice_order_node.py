#!/usr/bin/env python3
"""음성 주문 노드: 로봇이 /bungeoppang/listen 으로 주문을 요청하면 한 번 듣고,
알아들으면 {"counts": {"팥": n, "슈크림": m}}, 못 알아들으면 {"error": "..."} 를 /bungeoppang/order 로 보냄
  ros2 run bungeoppang voice_order                          # 마이크 (기본)
  ros2 run bungeoppang voice_order --ros-args -p mode:=text   # 키보드 입력 (테스트용)
"""
import json
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAX_ORDER = 6
HINT = "붕어빵 가게 주문. 메뉴: 팥붕, 슈붕, 팥 붕어빵, 슈크림 붕어빵."

# 속재료: 음성 인식이 자주 틀리는 표현까지 포함 (긴 단어 우선으로 찾음)
FILL_TABLE = sorted([(w, k) for k, ws in [
    ("슈크림", ["슈크림", "수크림", "슈크린", "쓔크림", "슈붕", "슈봉", "수붕", "수봉", "쓔붕", "슈뿡", "커스터드", "크림"]),
    ("팥", ["팥붕", "팥봉", "팟붕", "팟봉", "밭붕", "단팥", "팥소", "팥", "팟"]),
] for w in ws], key=lambda x: -len(x[0]))
NUM = {"하나": 1, "한": 1, "둘": 2, "두": 2, "셋": 3, "세": 3, "석": 3,
       "넷": 4, "네": 4, "넉": 4, "다섯": 5, "여섯": 6}
PURE_NUM = {"하나", "둘", "셋", "넷", "다섯", "여섯"}      # "개" 없이도 숫자로 확실한 말
NUM_RE = r"(\d+|하나|한|둘|두|셋|세|석|넷|네|넉|다섯|여섯|일곱|여덟|여덜|아홉|열)"
UNIT_RE = r"(?:개|게|계|마리)"
JOSA_RE = r"(?:붕어빵|붕어|어빵|빵|으로|로|은|는|이|가|을|를|도|랑|하고)*"


def to_num(s):
    return int(s) if s.isdigit() else NUM[s]


def scan(text, table):
    out, i = [], 0
    while i < len(text):
        for w, v in table:
            if text.startswith(w, i):
                out.append((i, v, w))
                i += len(w)
                break
        else:
            i += 1
    return out


def parse_counts(text):
    """문장 → {"팥": n, "슈크림": m}. 못 알아들으면 None"""
    t = re.sub(r"[\s,.!?~]", "", text)
    fills = scan(t, FILL_TABLE)
    if not fills:
        return None
    counts = {"팥": 0, "슈크림": 0}
    pending, each = [], None
    for n, (pi, kind, w) in enumerate(fills):
        end = fills[n + 1][0] if n + 1 < len(fills) else len(t)
        seg = t[pi + len(w):end]                                # 이 속재료 바로 뒤 ~ 다음 속재료 전
        if re.match(JOSA_RE + r"(?:빼|말고|없이|안|필요없)", seg):
            continue                                            # "슈크림은 빼고", "슈크림 말고"
        m = re.match(JOSA_RE + NUM_RE + "(" + UNIT_RE + ")?(씩)?", seg)
        if m and (m.group(2) or m.group(3) or m.group(1).isdigit() or m.group(1) in PURE_NUM):
            k = to_num(m.group(1))
            counts[kind] += k
            if m.group(3):
                each = k                                        # "두 개씩"
            continue
        mb = re.search(NUM_RE + UNIT_RE + r"(?:다|모두|전부)?$", t[max(0, pi - 8):pi])
        if mb and n == 0:
            counts[kind] += to_num(mb.group(1))                 # "세 개 다 팥"
        else:
            pending.append(kind)                                # 개수를 안 말한 종류
    for kind in pending:
        counts[kind] += each if each is not None else 1
    return counts if sum(counts.values()) > 0 else None


class VoiceOrder(Node):
    def __init__(self):
        super().__init__("bungeoppang_voice_order")
        self.mode = self.declare_parameter("mode", "voice").value
        self.model_size = self.declare_parameter("model", "small").value
        self.seconds = float(self.declare_parameter("seconds", 5.0).value)
        self.pub = self.create_publisher(String, "/bungeoppang/order", 10)
        self.create_subscription(String, "/bungeoppang/listen", self.on_listen, 10)
        self.trigger = threading.Event()
        self.prompt = ""
        self.model = None
        if self.mode == "voice":
            from faster_whisper import WhisperModel
            self.get_logger().info(f"음성 인식 모델({self.model_size}) 불러오는 중...")
            self.model = WhisperModel(self.model_size, device="cpu", compute_type="int8")

    def on_listen(self, msg):
        self.prompt = msg.data
        self.trigger.set()

    def hear(self):
        if self.mode != "voice":
            return input("   주문 입력 ▶ ").strip()
        import sounddevice as sd
        print(f"   🎤 듣는 중... ({self.seconds:.0f}초)")
        audio = sd.rec(int(self.seconds * 16000), samplerate=16000, channels=1, dtype="float32")
        sd.wait()
        segs, _ = self.model.transcribe(audio[:, 0], language="ko", beam_size=5,
                                        vad_filter=True, initial_prompt=HINT)
        text = "".join(s.text for s in segs).strip()
        print(f"   들은 말: \"{text}\"")
        return text

    def send(self, data):
        self.pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def loop(self):
        print("로봇의 주문 요청을 기다리는 중...")
        while rclpy.ok():
            if not self.trigger.wait(timeout=0.5):
                continue
            self.trigger.clear()
            print(f"\n🤖 {self.prompt}")
            time.sleep(0.3)
            text = self.hear()
            counts = parse_counts(text)
            if counts is None:
                print("   ✖ 팥/슈크림을 못 알아들었어요")
                self.send({"error": "not_understood", "text": text})
                continue
            total = sum(counts.values())
            if total > MAX_ORDER:
                print(f"   한 번에 최대 {MAX_ORDER}개까지라서 {MAX_ORDER}개로 맞출게요")
                over = total - MAX_ORDER
                for k in ("슈크림", "팥"):
                    cut = min(over, counts[k])
                    counts[k] -= cut
                    over -= cut
            print(f"   ✔ 알아들음: 팥 {counts['팥']}개, 슈크림 {counts['슈크림']}개")
            self.send({"counts": counts, "text": text})


def main():
    rclpy.init()
    node = VoiceOrder()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.loop()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()