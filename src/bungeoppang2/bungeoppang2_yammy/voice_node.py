"""음성인식 노드 (한국어, 오프라인)

마이크 → 말소리 구간 자동 감지 → faster-whisper로 한국어 인식
  - 인식된 문장 전체:  /bungeoppang/voice_text   (std_msgs/String)
  - 주문으로 해석되면:  /bungeoppang/order        (String, 예: "팥x2" / "슈크림x1")

설치 (한 번만):
  pip install faster-whisper sounddevice numpy --break-system-packages
  sudo apt install -y libportaudio2

실행:  ros2 run bungeoppang2 voice_node   (말 + 이 창에 문장 입력 둘 다 가능)
마이크 선택:  python3 -c "import sounddevice; print(sounddevice.query_devices())" 로 번호 확인 후
             아래 MIC_DEVICE에 번호 입력 (None = 기본 마이크)
"""
import queue
import re

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ================= ★ 조절하는 값 =================
MIC_DEVICE = None        # ★ 마이크 번호 (None = 기본). 안 잡히면 번호 지정
SAMPLE_RATE = 16000
MODEL_SIZE = "small"     # ★ tiny < base < small < medium (클수록 정확, 느림. RTX 4050은 small 권장)
USE_GPU = False          # ★ GPU 쓰려면 True (CUDA 라이브러리 필요)
VAD_THRESHOLD = 0.03     # ★ 말소리로 판단하는 소리 크기 (잡음이 주문으로 들리면 0.04~0.06)
SILENCE_SEC = 0.8        # ★ 이만큼 조용하면 문장이 끝난 것으로 판단
MAX_UTTER_SEC = 8.0      # ★ 한 문장 최대 길이
MIN_UTTER_SEC = 0.4      # 너무 짧은 소음은 버림
PRE_ROLL_BLOCKS = 5      # ★ 말 시작 전 0.5초도 함께 인식 (첫 글자 잘림 방지)
# 메뉴 단어를 미리 알려 주면 '팥'·'슈크림'을 훨씬 잘 알아들어요
HINT = "붕어빵 가게 주문입니다. 메뉴는 팥 붕어빵, 슈크림 붕어빵. 팥 하나, 팥 두 개, 슈크림 하나 주세요."
DECODE = dict(language="ko", beam_size=5, initial_prompt=None,
              vad_filter=False, condition_on_previous_text=False, temperature=0.0)

# ---- 주문 해석기 (예전 voice_order 노드의 강화판 이식) ----
# 발음 변형, 여러 종류 동시 주문("팥 둘, 슈크림 하나"), 빼기("슈크림은 빼고"), "~씩" 지원
FILL_TABLE = sorted([(w, k) for k, ws in [
    ("슈크림", ["슈크림", "수크림", "슈크린", "쓔크림", "치크림", "시크림", "쉬크림", "슈그림", "슈붕", "슈봉", "수붕", "수봉", "쓔붕", "슈뿡", "커스터드", "크림", "슈"]),
    ("팥", ["팥붕", "팥봉", "팟붕", "팟봉", "밭붕", "단팥", "팥소", "팥", "팟", "팝", "밭", "팍", "판", "파앗"]),
] for w in ws], key=lambda x: -len(x[0]))
NUM = {"하나": 1, "한": 1, "둘": 2, "두": 2, "셋": 3, "세": 3, "석": 3,
       "넷": 4, "네": 4, "넉": 4, "다섯": 5, "여섯": 6}
PURE_NUM = {"하나", "둘", "셋", "넷", "다섯", "여섯"}
NUM_RE = r"(\d+|하나|한|둘|두|셋|세|석|넷|네|넉|다섯|여섯)"
UNIT_RE = r"(?:개|게|계|마리)"
JOSA_RE = r"(?:붕어빵|붕어|어빵|빵|으로|로|은|는|이|가|을|를|도|랑|하고)*"
MAX_ORDER = 6            # ★ 한 번에 최대 개수
# =================================================


def _to_num(s):
    return int(s) if s.isdigit() else NUM[s]


def _scan(text, table):
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


def parse_order(text):
    """문장 → "팥x2,슈크림x1" 형식. 주문이 아니면 None"""
    t = re.sub(r"[\s,.!?~]", "", text)
    fills = _scan(t, FILL_TABLE)
    if not fills:
        return None
    counts = {"팥": 0, "슈크림": 0}
    pending, each = [], None
    for n, (pi, kind, w) in enumerate(fills):
        end = fills[n + 1][0] if n + 1 < len(fills) else len(t)
        seg = t[pi + len(w):end]
        if re.match(JOSA_RE + r"(?:빼|말고|없이|안|필요없)", seg):
            continue                                    # "슈크림은 빼고"
        m = re.match(JOSA_RE + NUM_RE + "(" + UNIT_RE + ")?(씩)?", seg)
        if m and (m.group(2) or m.group(3) or m.group(1).isdigit() or m.group(1) in PURE_NUM):
            k = _to_num(m.group(1))
            counts[kind] += k
            if m.group(3):
                each = k                                # "두 개씩"
            continue
        mb = re.search(NUM_RE + UNIT_RE + r"(?:다|모두|전부)?$", t[max(0, pi - 8):pi])
        if mb and n == 0:
            counts[kind] += _to_num(mb.group(1))        # "세 개 다 팥"
        else:
            pending.append(kind)
    for kind in pending:
        counts[kind] += each if each is not None else 1
    total = sum(counts.values())
    if total == 0:
        return None
    if total > MAX_ORDER:                               # 상한 넘으면 슈크림부터 줄임
        over = total - MAX_ORDER
        for k in ("슈크림", "팥"):
            cut = min(over, counts[k]); counts[k] -= cut; over -= cut
    return ",".join(f"{k}x{n}" for k, n in counts.items() if n > 0)


def _normalize(audio):
    """작게 녹음돼도 인식되게 음량을 맞춤"""
    peak = float(np.max(np.abs(audio))) or 1.0
    return (audio / peak * 0.9).astype(np.float32)


def _preload_cuda_libs(log):
    """pip로 설치한 nvidia-cublas-cu12 / nvidia-cudnn-cu12 라이브러리를 직접 로드.
    이러면 터미널마다 LD_LIBRARY_PATH를 export할 필요가 없어요."""
    import ctypes
    import glob
    import os
    try:
        import nvidia.cublas.lib as cub
        import nvidia.cudnn.lib as cud
    except ImportError:
        log.warning("nvidia-cublas-cu12 / nvidia-cudnn-cu12 미설치 → GPU 실패 시 CPU 사용")
        return
    dirs = [os.path.dirname(cub.__file__), os.path.dirname(cud.__file__)]
    order = ["libcublasLt.so*", "libcublas.so*", "libcudnn*.so*"]
    loaded = 0
    for pat in order:
        for d in dirs:
            for f in sorted(glob.glob(os.path.join(d, pat))):
                try:
                    ctypes.CDLL(f, mode=ctypes.RTLD_GLOBAL)
                    loaded += 1
                except OSError:
                    pass
    log.info(f"CUDA 라이브러리 {loaded}개 로드")


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_node")
        self.text_pub = self.create_publisher(String, "/bungeoppang/voice_text", 10)
        self.order_pub = self.create_publisher(String, "/bungeoppang/order", 10)
        self.listening = True                     # 총괄이 '지금 들어도 됨'일 때만 인식
        self.create_subscription(Bool, "/bungeoppang/listening",
                                 lambda m: setattr(self, "listening", m.data), 10)

        self.get_logger().info(f"Whisper '{MODEL_SIZE}' 모델 로딩 중...")
        if USE_GPU:
            _preload_cuda_libs(self.get_logger())
        from faster_whisper import WhisperModel
        try:
            self.model = WhisperModel(MODEL_SIZE, device="cuda" if USE_GPU else "cpu",
                                      compute_type="float16" if USE_GPU else "int8")
        except Exception as e:
            self.get_logger().warning(f"GPU 실패({e}) → CPU로 전환")
            self.model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
        self.get_logger().info("모델 준비 완료. 듣는 중...")

        self.q = queue.Queue()
        import sounddevice as sd
        self.stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32",
                                     device=MIC_DEVICE, blocksize=int(SAMPLE_RATE * 0.1),
                                     callback=lambda d, *_: self.q.put(d.copy()))
        self.stream.start()
        from collections import deque
        self.pre = deque(maxlen=PRE_ROLL_BLOCKS)
        self.buf = []
        self.silence = 0.0
        self.talking = False
        self.create_timer(0.05, self.loop)

    def loop(self):
        try:
            block = self.q.get_nowait()
        except queue.Empty:
            return
        if not self.listening:                    # 굽는 중엔 마이크 소리 버림
            self.buf, self.talking, self.silence = [], False, 0.0
            return
        level = float(np.sqrt(np.mean(block ** 2)))
        if level > VAD_THRESHOLD:
            if not self.talking:                       # 말 시작: 직전 0.5초도 함께
                self.buf = list(self.pre)
            self.talking = True
            self.silence = 0.0
            self.buf.append(block)
        elif self.talking:
            self.silence += len(block) / SAMPLE_RATE
            self.buf.append(block)
            if self.silence >= SILENCE_SEC or len(self.buf) * 0.1 >= MAX_UTTER_SEC:
                self.finish_utterance()
        if not self.talking:                           # 말 안 할 땐 최근 0.5초만 보관
            self.pre.append(block)

    def finish_utterance(self):
        audio = np.concatenate(self.buf).flatten()
        self.buf, self.talking, self.silence = [], False, 0.0
        if len(audio) / SAMPLE_RATE < MIN_UTTER_SEC:
            return
        audio = _normalize(audio)
        try:
            segments, _ = self.model.transcribe(audio, **DECODE)
            text = "".join(s.text for s in segments).strip()
        except RuntimeError as e:
            self.get_logger().warning(f"GPU 인식 실패({e}) → CPU로 전환해서 다시 인식")
            from faster_whisper import WhisperModel
            self.model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
            segments, _ = self.model.transcribe(audio, **DECODE)
            text = "".join(s.text for s in segments).strip()
        if not text:
            return
        self.get_logger().info(f"🎤 인식: {text}")
        self.text_pub.publish(String(data=text))
        order = parse_order(text)          # 예: "팥x2,슈크림x1"
        if order:
            self.get_logger().info(f"🧾 주문: {order}")
            self.order_pub.publish(String(data=order))


class TextOrderNode(Node):
    """(임시) 음성인식 대신 키보드로 문장 입력.
    마이크 노드와 똑같이 /bungeoppang/voice_text + /bungeoppang/order 를 발행해서
    총괄 노드 입장에서는 말로 주문한 것과 똑같아요 (인사 → 끄덕/도리도리 → 굽기)."""

    def __init__(self):
        super().__init__("text_order_node")
        self.text_pub = self.create_publisher(String, "/bungeoppang/voice_text", 10)
        self.order_pub = self.create_publisher(String, "/bungeoppang/order", 10)
        self.listening = True
        self.create_subscription(Bool, "/bungeoppang/listening",
                                 lambda m: setattr(self, "listening", m.data), 10)

    def submit(self, text):
        submit_text(self, text)


def submit_text(node, text):
    """키보드로 친 문장을 말로 한 것과 똑같이 발행 (voice_node·text_order 공용)"""
    text = text.strip()
    if not text:
        return
    if not node.listening:
        print("   (지금은 굽는 중이라 주문을 안 받아요 — 끝나면 다시 입력하세요)")
        return
    node.get_logger().info(f"⌨ 입력: {text}")
    node.text_pub.publish(String(data=text))
    order = parse_order(text)
    if order:
        node.get_logger().info(f"🧾 주문: {order}")
        node.order_pub.publish(String(data=order))
    else:
        node.get_logger().info("   주문으로 해석 안 됨 → 도리도리 (예: '팥 두 개 주세요', '슈붕 하나')")


def _keyboard_loop(node):
    """마이크와 함께 쓰는 키보드 입력 (Enter로 보냄)"""
    try:
        while rclpy.ok():
            submit_text(node, input())
    except (EOFError, KeyboardInterrupt):
        pass


def main_text(args=None):
    """실행: ros2 run bungeoppang2 text_order   (launch는 voice:=false 로)"""
    import threading
    rclpy.init(args=args)
    node = TextOrderNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print("⌨ 텍스트 주문 모드. 문장을 입력하고 Enter (예: 안녕하세요 / 팥 하나 슈크림 하나 주세요). 종료: Ctrl+C")
    try:
        while rclpy.ok():
            node.submit(input("주문> "))
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """음성 + 텍스트 동시: 마이크로 말해도 되고, 이 창에 문장을 치고 Enter 해도 돼요.
    마이크/Whisper가 안 되면 텍스트만으로 계속 받아요."""
    import threading
    rclpy.init(args=args)
    voice = True
    try:
        node = VoiceNode()
    except Exception as e:                       # 마이크 없음, faster-whisper 미설치 등
        voice = False
        node = TextOrderNode()
        node.get_logger().warning(f"음성 인식을 못 켰어요({e}) → 텍스트로만 주문 받아요")
    threading.Thread(target=_keyboard_loop, args=(node,), daemon=True).start()
    print(("🎤 말로 주문하거나, " if voice else "") +
          "⌨ 이 창에 문장을 입력하고 Enter (예: 팥 하나 슈크림 하나 주세요). 종료: Ctrl+C", flush=True)
    try:
        rclpy.spin(node)
    except Exception:                            # Ctrl+C·창 닫기로 ROS가 먼저 꺼져도 조용히 종료
        pass
    except KeyboardInterrupt:
        pass
    if voice:
        node.stream.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
