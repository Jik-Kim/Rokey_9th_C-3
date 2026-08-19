"""
fake_camera — Isaac Sim 없이 색상 감지 파이프라인을 자체 검증한다.

    ros2 run m0609 fake_camera

Isaac Sim 이 안 뜨거나 GPU 노트북이 없을 때, 손목 카메라 대신
이 노드가 /rgb 를 만들어 쏜다. 그리고 /color_id 를 되받아 채점한다.

    fake_camera  --/rgb-------->  m0609_color_detector
    fake_camera  <--/color_id---  m0609_color_detector

무슨 색을 그렸는지 아는 쪽이 채점하므로 정답을 알고 있다.
Isaac Sim 에서 7_pick_place_color.py 를 띄우기 전에 PC B 쪽이
제대로 도는지 여기서 먼저 확인한다.

장면(scene)은 아래 SCENES 순서대로 돌아간다. 각 장면마다
  1. 몇 초 동안 같은 그림을 계속 발행하고
  2. 끝나기 직전 구간의 /color_id 를 모아
  3. 기대값과 같은지 PASS / FAIL 로 찍는다

큐브가 작은 장면은 min_pixel_ratio 를, 그림자 장면은 V 하한을 시험한다.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


# ══════════════════════════════════════════════════════════════
#  색상 코드 — 감지 노드와 반드시 같아야 한다
# ══════════════════════════════════════════════════════════════
COLOR_NONE  = 0
COLOR_BLUE  = 1
COLOR_GREEN = 2

COLOR_NAMES = {COLOR_NONE: "NONE", COLOR_BLUE: "BLUE", COLOR_GREEN: "GREEN"}

# 그릴 색 (B, G, R) — 7_pick_place_color.py 의 큐브 색과 맞춘다
CUBE_BGR = {
    COLOR_BLUE:  (242,  38,  13),
    COLOR_GREEN: ( 38, 217,  13),
}


# ══════════════════════════════════════════════════════════════
#  영상 설정 — Isaac Sim 손목 카메라와 같은 규격
# ══════════════════════════════════════════════════════════════
WIDTH  = 640
HEIGHT = 640
FPS    = 30.0

# 배경 회색 — 시뮬 바닥/테이블이 대략 이 밝기다 (RGB 200 언저리)
BG_GRAY  = 202
BG_SLOPE = 18       # 위아래 밝기 차 — 조명 기울기 흉내
NOISE_STD = 3.0     # 센서 잡음


# ══════════════════════════════════════════════════════════════
#  장면 목록
#    (설명, 색, 큐브 변 길이 비율, 중심 x 비율, 중심 y 비율, 그림자)
#    변 길이 비율 0.06 → 38px → 면적비 0.0036.  min_pixel_ratio 0.002 의 바로 위다
# ══════════════════════════════════════════════════════════════
SCENES = [
    ("빈 화면",        COLOR_NONE,  0.00, 0.50, 0.50, False),
    ("파랑 멀리",      COLOR_BLUE,  0.06, 0.50, 0.50, False),
    ("파랑 가까이",    COLOR_BLUE,  0.30, 0.48, 0.52, True),
    ("파랑 구석",      COLOR_BLUE,  0.12, 0.20, 0.75, True),
    ("빈 화면",        COLOR_NONE,  0.00, 0.50, 0.50, False),
    ("초록 멀리",      COLOR_GREEN, 0.06, 0.50, 0.50, False),
    ("초록 가까이",    COLOR_GREEN, 0.30, 0.52, 0.48, True),
    ("초록 구석",      COLOR_GREEN, 0.12, 0.78, 0.25, True),
]

HOLD_SEC  = 3.0     # 장면 하나를 유지하는 시간
JUDGE_SEC = 1.0     # 마지막 이만큼의 /color_id 로 채점한다


class FakeCamera(Node):

    def __init__(self):
        super().__init__("m0609_fake_camera")

        self.declare_parameter("image_topic",  "/rgb")
        self.declare_parameter("result_topic", "/color_id")
        self.declare_parameter("fps",          FPS)
        self.declare_parameter("hold_sec",     HOLD_SEC)
        self.declare_parameter("judge_sec",    JUDGE_SEC)
        self.declare_parameter("loops",        1)       # 0 이면 무한 반복
        self.declare_parameter("noise_std",    NOISE_STD)
        self.declare_parameter("show_window",  False)

        p = self.get_parameter
        image_topic  = p("image_topic").value
        result_topic = p("result_topic").value
        fps          = float(p("fps").value)

        self._bridge = CvBridge()

        # Isaac Sim 과 같은 센서 QoS 로 쏜다.  감지 노드가 그렇게 구독한다
        self._pub = self.create_publisher(Image, image_topic, qos_profile_sensor_data)
        self.create_subscription(Int32, result_topic, self._on_color, 10)

        # 채점 상태
        self._scene    = 0
        self._loop     = 0
        self._t        = 0.0            # 현재 장면에서 흐른 시간
        self._votes    = []             # 채점 구간에 받은 color_id
        self._last_id  = COLOR_NONE
        self._got_any  = False
        self._pass     = 0
        self._fail     = 0
        self._done     = False

        self._dt = 1.0 / fps
        self.create_timer(self._dt, self._tick)

        self._log_header(image_topic, result_topic, fps)
        self._announce()

    # ── 출력 ────────────────────────────────────────────
    def _log_header(self, image_topic, result_topic, fps):
        p = self.get_parameter
        self.get_logger().info(
            "\n"
            f"   publish      {image_topic}   {WIDTH}x{HEIGHT} rgb8 @ {fps:g} fps\n"
            f"   subscribe    {result_topic}  (0=NONE 1=BLUE 2=GREEN)\n"
            f"   scenes       {len(SCENES)} 개  x {p('loops').value or '무한'} 회\n"
            f"   hold/judge   {p('hold_sec').value}s / {p('judge_sec').value}s\n"
            "\n"
            "   다른 터미널에서 감지 노드를 함께 띄운다\n"
            "       ros2 run m0609 m0609_color_detector"
        )

    def _announce(self):
        name, color, size, _, _, _ = SCENES[self._scene]
        self.get_logger().info(
            f"[{self._scene + 1}/{len(SCENES)}] {name}"
            f"   기대 color_id = {color} ({COLOR_NAMES[color]})"
            f"   큐브 {int(size * WIDTH)}px"
        )

    # ── 그림 ────────────────────────────────────────────
    def _background(self):
        """위에서 아래로 살짝 밝아지는 회색 판에 잡음을 얹는다"""
        col = np.linspace(BG_GRAY - BG_SLOPE, BG_GRAY + BG_SLOPE, HEIGHT)
        img = np.repeat(col[:, None], WIDTH, axis=1)
        img = np.repeat(img[:, :, None], 3, axis=2)

        std = float(self.get_parameter("noise_std").value)
        if std > 0:
            img = img + np.random.normal(0.0, std, img.shape)

        return np.clip(img, 0, 255).astype(np.uint8)

    def _render(self):
        """현재 장면을 BGR 이미지로 그린다"""
        img = self._background()
        _, color, size, cx, cy, shadow = SCENES[self._scene]

        if color == COLOR_NONE or size <= 0.0:
            return img

        half = int(size * WIDTH / 2)
        x, y = int(cx * WIDTH), int(cy * HEIGHT)
        x0, y0 = max(0, x - half), max(0, y - half)
        x1, y1 = min(WIDTH, x + half), min(HEIGHT, y + half)

        # 그림자 — 큐브 오른쪽 아래로 어두운 회색. V 하한이 이걸 걸러야 한다
        if shadow:
            off = max(2, half // 3)
            sx0, sy0 = min(WIDTH - 1, x0 + off), min(HEIGHT - 1, y0 + off)
            sx1, sy1 = min(WIDTH, x1 + off), min(HEIGHT, y1 + off)
            img[sy0:sy1, sx0:sx1] = (img[sy0:sy1, sx0:sx1] * 0.55).astype(np.uint8)

        # 큐브 — 윗면은 밝고 앞면은 어둡게. 색상(H)은 그대로 두고 명도(V)만 바꾼다
        face = np.zeros((y1 - y0, x1 - x0, 3), dtype=np.float64)
        face[:, :] = CUBE_BGR[color]

        shade = np.linspace(1.0, 0.72, face.shape[0])[:, None, None]
        face = face * shade

        img[y0:y1, x0:x1] = np.clip(face, 0, 255).astype(np.uint8)
        return img

    # ── 콜백 ────────────────────────────────────────────
    def _on_color(self, msg):
        self._last_id = int(msg.data)
        self._got_any = True

    def _tick(self):
        if self._done:
            return

        img = self._render()

        # 감지 노드는 bgr8 로 변환해 받는다. Isaac Sim 과 같은 rgb8 로 쏜다
        msg = self._bridge.cv2_to_imgmsg(
            cv2.cvtColor(img, cv2.COLOR_BGR2RGB), encoding="rgb8"
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wrist_camera"
        self._pub.publish(msg)

        if self.get_parameter("show_window").value:
            cv2.imshow("fake_camera", img)
            cv2.waitKey(1)

        self._advance()

    # ── 채점 ────────────────────────────────────────────
    def _advance(self):
        """시간을 흘리고, 채점 구간에 들어오면 표를 모은다"""
        hold  = float(self.get_parameter("hold_sec").value)
        judge = float(self.get_parameter("judge_sec").value)

        self._t += self._dt

        if self._t >= hold - judge:
            self._votes.append(self._last_id)

        if self._t < hold:
            return

        self._score()
        self._next_scene()

    def _score(self):
        """표의 최빈값을 결과로 본다. 한두 프레임 튀는 것은 넘어간다"""
        name, expect = SCENES[self._scene][0], SCENES[self._scene][1]

        if not self._votes:
            got, ratio = COLOR_NONE, 0.0
        else:
            values, counts = np.unique(self._votes, return_counts=True)
            got   = int(values[np.argmax(counts)])
            ratio = float(counts.max()) / len(self._votes)

        ok = (got == expect)
        self._pass += int(ok)
        self._fail += int(not ok)

        tag = "PASS" if ok else "FAIL"
        line = (f"   {tag}  {name}"
                f"   기대 {COLOR_NAMES[expect]:5s}"
                f"   수신 {COLOR_NAMES.get(got, got):5s}"
                f"   ({ratio * 100:.0f}% / {len(self._votes)} 프레임)")

        if ok:
            self.get_logger().info(line)
        else:
            self.get_logger().error(line)

        self._votes = []

    def _next_scene(self):
        self._t = 0.0
        self._scene += 1

        if self._scene < len(SCENES):
            self._announce()
            return

        self._scene = 0
        self._loop += 1

        loops = int(self.get_parameter("loops").value)
        if loops and self._loop >= loops:
            self._finish()
        else:
            self._announce()

    def _finish(self):
        self._done = True
        total = self._pass + self._fail

        if not self._got_any:
            self.get_logger().error(
                "\n"
                "   /color_id 를 한 번도 못 받았다.\n"
                "   감지 노드가 떠 있는지, ROS_DOMAIN_ID 가 같은지 확인한다\n"
                "       ros2 topic list | grep color\n"
                "       ros2 run m0609 m0609_color_detector"
            )
        else:
            self.get_logger().info(
                f"\n   결과   PASS {self._pass} / {total}   FAIL {self._fail}"
            )

        raise SystemExit(0 if (self._got_any and self._fail == 0) else 1)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:                         # _finish 가 던진다
        code = int(e.code or 0)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    main()
