"""
color_tune — HSV 임계값을 눈으로 보며 맞추는 도구.

    ros2 run m0609 color_tune

bag 을 재생해 놓고 트랙바를 돌리면 마스크가 실시간으로 바뀐다.
값이 정해지면 창 위에 찍힌 문장을 그대로 복사해
m0609_color_detector 실행 인자로 넘기면 된다.

    q   종료
    p   현재 값 터미널에 출력
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


WINDOW = "color_tune"

# (트랙바 이름, 초기값, 최대값)
BARS = [
    ("H min",  40, 179),
    ("H max",  85, 179),
    ("S min",  80, 255),
    ("S max", 255, 255),
    ("V min",  50, 255),
    ("V max", 255, 255),
    ("ROI %",  50, 100),
]


class ColorTune(Node):

    def __init__(self):
        super().__init__("color_tune")
        self.declare_parameter("image_topic", "/rgb")
        topic = self.get_parameter("image_topic").value

        self._bridge = CvBridge()
        self._frame = None

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        for name, init, maxv in BARS:
            cv2.createTrackbar(name, WINDOW, init, maxv, lambda _v: None)

        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.create_timer(0.03, self._render)

        self.get_logger().info(f"subscribe {topic} — 트랙바를 돌려 임계값을 맞추세요 (q 종료)")

    def _on_image(self, msg):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")

    def _values(self):
        return [cv2.getTrackbarPos(name, WINDOW) for name, _, _ in BARS]

    def _render(self):
        if self._frame is None:
            return

        hmin, hmax, smin, smax, vmin, vmax, roi_pct = self._values()

        bgr = self._frame
        h, w = bgr.shape[:2]
        r = max(roi_pct, 5) / 100.0
        rw, rh = int(w * r), int(h * r)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        roi = bgr[y0:y0 + rh, x0:x0 + rw]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([hmin, smin, vmin], np.uint8),
                           np.array([hmax, smax, vmax], np.uint8))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        ratio = float(cv2.countNonZero(mask)) / (mask.shape[0] * mask.shape[1])

        # 왼쪽 원본(ROI 표시) | 오른쪽 마스크
        left = bgr.copy()
        cv2.rectangle(left, (x0, y0), (x0 + rw, y0 + rh), (255, 255, 255), 1)

        right = np.zeros_like(bgr)
        right[y0:y0 + rh, x0:x0 + rw] = cv2.bitwise_and(roi, roi, mask=mask)

        view = np.hstack([left, right])
        cv2.putText(view, f"ratio {ratio:.4f}   HSV [{hmin},{smin},{vmin}] ~ [{hmax},{smax},{vmax}]",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt
        if key == ord("p"):
            self._print(hmin, hmax, smin, smax, vmin, vmax, roi_pct, ratio)

    def _print(self, hmin, hmax, smin, smax, vmin, vmax, roi_pct, ratio):
        print(f"\n  ratio {ratio:.4f}   ROI {roi_pct}%")
        print(f"  lower [{hmin}, {smin}, {vmin}]   upper [{hmax}, {smax}, {vmax}]")
        print( "  ros2 run m0609 m0609_color_detector --ros-args \\")
        print(f"      -p roi_ratio:={roi_pct / 100.0} \\")
        print(f"      -p green_lower:=\"[{hmin}, {smin}, {vmin}]\" \\")
        print(f"      -p green_upper:=\"[{hmax}, {smax}, {vmax}]\"\n")


def main(args=None):
    rclpy.init(args=args)
    node = ColorTune()
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
