# m0609 — 색상 감지 노드 (PC B)

Isaac Sim(PC A)의 손목 카메라 영상을 받아 큐브 색상을 판별해 발행한다.

| 항목 | 값 |
|---|---|
| 구독 | `/rgb` `sensor_msgs/Image` (640×640, rgb8) |
| 발행 | `/color_id` `std_msgs/Int32` — 0=없음, 1=파랑, 2=초록 |
| 발행 | `/color_debug` `sensor_msgs/Image` — ROI·마스크 오버레이 |

> `/color_id` 이름과 코드값(1/2)은 PC A의 `7_pick_place_color.py` 구독부와 반드시 일치해야 한다.

| 노드 | 하는 일 |
|---|---|
| `m0609_color_detector` | `/rgb` 를 보고 `/color_id` 를 발행한다 (본체) |
| `color_tune` | 트랙바로 HSV 임계값을 맞춘다 |
| `fake_camera` | Isaac Sim 없이 `/rgb` 를 합성해 쏘고 `/color_id` 를 채점한다 |

## 빌드

```bash
cd ~/cobot3_ws
colcon build --packages-select m0609
source install/setup.bash
```

## ROS 네트워크 (수업 기준)

```bash
export ROS_DOMAIN_ID=50
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros_set
```

## 0) Isaac Sim 없이 먼저 확인하기 — `fake_camera`

Isaac Sim(PC A)이 안 뜨면 bag 도 못 뜬다. 그때는 `fake_camera` 가 손목 카메라
대신 합성 영상을 `/rgb` 로 쏘고, 되돌아온 `/color_id` 를 스스로 채점한다.
무슨 색을 그렸는지 아는 쪽이 채점하므로 정답을 안다.

```bash
# 터미널 1
ros2 run m0609 m0609_color_detector

# 터미널 2
ros2 run m0609 fake_camera
```

장면 8개(빈 화면 / 멀리 / 가까이 / 구석 × 파랑·초록)를 3초씩 돌며 찍는다.

```
[2/8] 파랑 멀리   기대 color_id = 1 (BLUE)   큐브 38px
   PASS  파랑 멀리   기대 BLUE    수신 BLUE    (100% / 30 프레임)
...
   결과   PASS 8 / 8   FAIL 0
```

- `FAIL` 이 나오면 그 장면 조건에서 HSV 임계값이나 `min_pixel_ratio` 가 안 맞는 것이다
- `/color_id 를 한 번도 못 받았다` 가 뜨면 감지 노드나 ROS_DOMAIN_ID 문제다
- `ros2 run m0609 color_tune` 도 이 합성 영상에 그대로 붙는다

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `image_topic` | `/rgb` | 발행 토픽 |
| `result_topic` | `/color_id` | 채점용 구독 토픽 |
| `fps` | `30.0` | 발행 주기 |
| `hold_sec` | `3.0` | 장면 하나를 유지하는 시간 |
| `judge_sec` | `1.0` | 장면 끝 이 구간의 `/color_id` 로 채점 |
| `loops` | `1` | 전체 장면을 몇 바퀴 돌지. `0` 이면 무한 |
| `noise_std` | `3.0` | 센서 잡음 세기 |
| `show_window` | `false` | 합성 영상을 cv2 창으로 보기 |

> 어디까지나 합성 영상이다. 여기서 PASS 한다고 실제 렌더링 영상에서도 맞는다는
> 보장은 없다. Isaac Sim 이 뜨면 아래 bag 절차로 다시 맞춘다.

## 1) bag 저장 — Isaac Sim 을 켜 놓고

```bash
ros2 topic hz /rgb          # 발행 확인
mkdir -p ~/cobot3_ws/bags && cd ~/cobot3_ws/bags

# raw 이미지는 640×640×3 ≈ 1.2MB/frame → 30fps 면 초당 35MB. 반드시 압축한다
ros2 bag record /rgb /camera_info \
    -o cube_color \
    --compression-mode file --compression-format zstd
```

큐브 색이 바뀔 때마다 몇 초씩, 파랑/초록 둘 다 담기게 녹화한다.

## 2) bag 재생하며 임계값 맞추기

```bash
# 터미널 1
ros2 bag play ~/cobot3_ws/bags/cube_color --loop

# 터미널 2 — 트랙바로 HSV 조정, p 키로 현재 값 출력, q 종료
ros2 run m0609 color_tune
```

## 3) 감지 노드 확인

```bash
# 터미널 1
ros2 bag play ~/cobot3_ws/bags/cube_color --loop

# 터미널 2
ros2 run m0609 m0609_color_detector

# 터미널 3
ros2 topic echo /color_id
rqt   # Plugins > Visualization > Image View > /color_debug
```

## 파라미터 — `m0609_color_detector`

| 이름 | 기본값 | 설명 |
|---|---|---|
| `image_topic` | `/rgb` | 구독 토픽 |
| `result_topic` | `/color_id` | 발행 토픽 |
| `roi_ratio` | `1.0` | 중앙 ROI 비율. 마커까지 보이면 줄인다 |
| `min_pixel_ratio` | `0.002` | ROI 면적 대비 최소 검출 비율 |
| `stable_count` | `3` | 연속 몇 프레임 같아야 확정할지 |
| `blue_lower` / `blue_upper` | `[100,80,50]` / `[130,255,255]` | 파랑 HSV 범위 |
| `green_lower` / `green_upper` | `[40,80,50]` / `[85,255,255]` | 초록 HSV 범위 |
| `show_window` | `false` | cv2 창 직접 띄우기 |

```bash
ros2 run m0609 m0609_color_detector --ros-args \
    -p roi_ratio:=0.5 \
    -p blue_lower:="[95, 120, 60]"
```

### `roi_ratio` 를 왜 줄이는가

손목 카메라는 RealSense D455, 화각 **90.53°** 다 (640×640 이라 가로세로 같다).
`7_pick_place_color.py` 의 WAIT_COLOR 자세에서 카메라는 바닥 위 0.394 m 에 있고
**한 변 0.80 m** 를 본다. 바닥에 깔린 색 마커가 이 안에 들어오면,
마커(0.08 m)가 큐브 윗면(0.05 m)보다 커서 픽셀 수 비교에서 마커가 이겨 버린다.

7번 코드는 마커를 `y = -0.45` 로 빼서 화면 밖으로 내보냈다(측정으로 확인).
`roi_ratio 0.5` 는 그 위에 얹는 이중 안전장치다 — 카메라 중심에서 ±0.199 m 만 보는데,
큐브는 중심에서 최대 0.07 m 안에 들어오므로 잘릴 일이 없다.

> 씬 배치를 바꾸면 이 숫자들도 다시 재야 한다.
