"""
m0609_color_detector — 손목 카메라 영상에서 큐브 색상을 판별해 발행한다.

    ros2 run m0609 m0609_color_detector

구독  /rgb        sensor_msgs/Image     Isaac Sim 손목 카메라 (640x640, rgb8)
발행  /color_id   std_msgs/Int32        0=없음  1=파랑  2=초록
발행  /color_debug sensor_msgs/Image    ROI·마스크를 그린 디버그 영상

판별 흐름
  1. 이미지 중앙 ROI 만 잘라낸다.  주변 마커까지 보면 오판한다
  2. BGR -> HSV.  조명 변화에 강한 건 H(색상) 채널이다
  3. 파랑/초록 마스크를 각각 만들고 픽셀 수를 센다
  4. 둘 중 많은 쪽이 최소 비율을 넘으면 그 색으로 본다
  5. 같은 결과가 STABLE_COUNT 번 연속 나와야 확정한다 (깜빡임 방지)
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
#  색상 코드 — Isaac Sim 쪽(7_pick_place_color.py)과 반드시 같아야 한다
# ══════════════════════════════════════════════════════════════
COLOR_NONE  = 0
COLOR_BLUE  = 1
COLOR_GREEN = 2

COLOR_NAMES = {COLOR_NONE: "NONE", COLOR_BLUE: "BLUE", COLOR_GREEN: "GREEN"}


# ══════════════════════════════════════════════════════════════
#  HSV 임계값 (OpenCV 기준: H 0~179, S 0~255, V 0~255)
# ══════════════════════════════════════════════════════════════
# 파랑  H 100~130  — 하늘색부터 남색까지
# 초록  H  40~ 85  — 연두부터 청록 직전까지
# S 하한을 올리면 회색·흰색 배경이 걸러지고
# V 하한을 올리면 그림자가 걸러진다
BLUE_LOWER  = [100,  80,  50]
BLUE_UPPER  = [130, 255, 255]
GREEN_LOWER = [ 40,  80,  50]
GREEN_UPPER = [ 85, 255, 255]

# ROI — 이미지 중앙에서 잘라낼 비율 (1.0 이면 전체)
# 0.5 로는 초록 큐브가 화면 좌상단(중심 x0.16 y0.33)에 놓여 ROI 밖으로 벗어난다.
# 씬에 색 있는 마커가 추가되면 다시 줄이고 ROI 중심을 파라미터로 빼야 한다
ROI_RATIO = 1.0

# ROI 면적 대비 이 비율을 넘어야 "큐브가 있다"고 본다
# 손목 카메라가 먼 자세일 때 큐브는 640x640 중 40px 남짓(≈0.003)까지 작아진다.
# 접근하면 훨씬 커지므로 이 값은 하한선으로 잡는다
MIN_PIXEL_RATIO = 0.002

# 같은 판별이 이만큼 연속되어야 확정한다
STABLE_COUNT = 3


class ColorDetector(Node):

    def __init__(self):
        super().__init__("m0609_color_detector")

        # ── 파라미터 — 런타임에 튜닝할 수 있도록 전부 노출한다 ──
        self.declare_parameter("image_topic",     "/rgb")
        self.declare_parameter("result_topic",    "/color_id")
        self.declare_parameter("debug_topic",     "/color_debug")
        self.declare_parameter("roi_ratio",       ROI_RATIO)
        self.declare_parameter("min_pixel_ratio", MIN_PIXEL_RATIO)
        self.declare_parameter("stable_count",    STABLE_COUNT)
        self.declare_parameter("blue_lower",      BLUE_LOWER)
        self.declare_parameter("blue_upper",      BLUE_UPPER)
        self.declare_parameter("green_lower",     GREEN_LOWER)
        self.declare_parameter("green_upper",     GREEN_UPPER)
        self.declare_parameter("publish_debug",   True)
        self.declare_parameter("show_window",     False)

        p = self.get_parameter
        image_topic  = p("image_topic").value
        result_topic = p("result_topic").value
        debug_topic  = p("debug_topic").value

        self._bridge = CvBridge()

        # 이미지는 센서 QoS(BEST_EFFORT).  Isaac Sim 실시간 발행과 bag 재생 양쪽에 맞는다
        self._sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(Int32, result_topic, 10)
        self._dbg = self.create_publisher(Image, debug_topic, 10)

        # 판별 상태
        self._last_raw   = COLOR_NONE   # 이번 프레임의 즉석 판별
        self._stable     = COLOR_NONE   # 확정된 값
        self._streak     = 0
        self._frames     = 0

        self._log_header(image_topic, result_topic, debug_topic)

    # ── 출력 ────────────────────────────────────────────
    def _log_header(self, image_topic, result_topic, debug_topic):
        p = self.get_parameter
        self.get_logger().info(
            "\n"
            f"   subscribe    {image_topic}\n"
            f"   publish      {result_topic}   (0=NONE 1=BLUE 2=GREEN)\n"
            f"   debug image  {debug_topic}\n"
            f"   roi ratio    {p('roi_ratio').value}\n"
            f"   min pixels   {p('min_pixel_ratio').value}\n"
            f"   blue  HSV    {list(p('blue_lower').value)} ~ {list(p('blue_upper').value)}\n"
            f"   green HSV    {list(p('green_lower').value)} ~ {list(p('green_upper').value)}"
        )

    # ── ROI ─────────────────────────────────────────────
    def _roi_box(self, h, w):
        """중앙 ROI 의 (x0, y0, x1, y1). 비율 1.0 이면 전체 이미지"""
        r = float(self.get_parameter("roi_ratio").value)
        r = min(max(r, 0.05), 1.0)
        rw, rh = int(w * r), int(h * r)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        return x0, y0, x0 + rw, y0 + rh

    # ── 마스크 ──────────────────────────────────────────
    def _mask(self, hsv, lower_name, upper_name):
        """HSV 범위로 이진 마스크를 만들고 잡음을 제거한다"""
        lower = np.array(self.get_parameter(lower_name).value, dtype=np.uint8)
        upper = np.array(self.get_parameter(upper_name).value, dtype=np.uint8)
        m = cv2.inRange(hsv, lower, upper)

        # 열기(작은 점 제거) -> 닫기(구멍 메우기)
        k = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    # ── 콜백 ────────────────────────────────────────────
    def _on_image(self, msg):
        try:
            # Isaac Sim 은 rgb8 로 보낸다. bgr8 을 요구하면 cv_bridge 가 변환해 준다
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        h, w = bgr.shape[:2]
        x0, y0, x1, y1 = self._roi_box(h, w)
        roi = bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        blue_mask  = self._mask(hsv, "blue_lower",  "blue_upper")
        green_mask = self._mask(hsv, "green_lower", "green_upper")

        area = roi.shape[0] * roi.shape[1]
        blue_ratio  = float(cv2.countNonZero(blue_mask))  / area
        green_ratio = float(cv2.countNonZero(green_mask)) / area

        raw = self._decide(blue_ratio, green_ratio)
        self._update_stable(raw)

        # 확정값을 매 프레임 발행한다.  구독자가 늦게 붙어도 바로 받는다
        self._pub.publish(Int32(data=self._stable))

        self._frames += 1
        if self._frames % 30 == 0:
            self.get_logger().info(
                f"blue {blue_ratio:6.3f}  green {green_ratio:6.3f}"
                f"   raw {COLOR_NAMES[raw]:5s}  ->  {COLOR_NAMES[self._stable]}"
            )

        if self.get_parameter("publish_debug").value:
            self._publish_debug(bgr, (x0, y0, x1, y1), blue_mask, green_mask,
                                blue_ratio, green_ratio, msg.header)

    def _decide(self, blue_ratio, green_ratio):
        """더 많이 잡힌 쪽을 고르되, 최소 비율을 못 넘으면 NONE"""
        min_ratio = float(self.get_parameter("min_pixel_ratio").value)
        if max(blue_ratio, green_ratio) < min_ratio:
            return COLOR_NONE
        return COLOR_BLUE if blue_ratio >= green_ratio else COLOR_GREEN

    def _update_stable(self, raw):
        """같은 판별이 연속으로 나와야 확정값을 바꾼다"""
        need = int(self.get_parameter("stable_count").value)

        if raw == self._last_raw:
            self._streak += 1
        else:
            self._last_raw = raw
            self._streak = 1

        if self._streak >= need and self._stable != raw:
            self.get_logger().info(
                f"색상 확정: {COLOR_NAMES[self._stable]} -> {COLOR_NAMES[raw]}  ({raw})"
            )
            self._stable = raw

    # ── 디버그 영상 ─────────────────────────────────────
    def _publish_debug(self, bgr, box, blue_mask, green_mask,
                       blue_ratio, green_ratio, header):
        """원본에 ROI 사각형과 검출 마스크 윤곽, 수치를 얹는다"""
        x0, y0, x1, y1 = box
        out = bgr.copy()

        cv2.rectangle(out, (x0, y0), (x1, y1), (255, 255, 255), 1)

        # 마스크는 ROI 좌표계이므로 원본 위치로 되돌려 칠한다
        overlay = out[y0:y1, x0:x1]
        overlay[blue_mask  > 0] = (255, 0, 0)
        overlay[green_mask > 0] = (0, 255, 0)

        label = COLOR_NAMES[self._stable]
        color = {COLOR_BLUE: (255, 0, 0),
                 COLOR_GREEN: (0, 255, 0)}.get(self._stable, (200, 200, 200))
        cv2.putText(out, f"{label} ({self._stable})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(out, f"blue {blue_ratio:.3f}  green {green_ratio:.3f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        msg = self._bridge.cv2_to_imgmsg(out, encoding="bgr8")
        msg.header = header
        self._dbg.publish(msg)

        if self.get_parameter("show_window").value:
            cv2.imshow("m0609_color_detector", out)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
