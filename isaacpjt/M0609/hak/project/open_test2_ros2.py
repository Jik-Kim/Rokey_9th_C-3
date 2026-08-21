# test2.usd 를 열고 ROS2 브리지를 붙인다.
#
#   cd ~/isaacsim && ./isaac-sim.sh --exec ~/cobot3_ws/isaacpjt/M0609/hak/project/open_test2_ros2.py
#   (또는 같은 폴더의 run_open_test2_ros2.sh)
#
# test2.usd 에는 ROS2 그래프가 없다. 컨베이어 18조각과 바닥, 큐브 2개뿐이라
# 발행할 게 아직 없어서, 브리지가 살아 있는지 확인할 수 있는 최소 그래프를
# 런타임에 만든다:  /World/ROS2Clock  ->  /clock  (rosgraph_msgs/Clock)
#
# 스테이지에는 저장하지 않는다. test2.usd 는 소터 넣기 전 원본 라인이라
# 그대로 두고, 그래프는 열 때마다 다시 만든다.
#
# OnPlaybackTick 은 Play 중에만 돌기 때문에 타임라인을 자동으로 시작한다.
# 멈춘 채로 띄우려면 PL_NOPLAY=1 을 준다.
import os

import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.join(HERE, "project_1", "test2.usd")

GRAPH_PATH = "/World/ROS2Clock"

enable_extension("isaacsim.core.nodes")
enable_extension("isaacsim.ros2.bridge")

import omni.graph.core as og  # noqa: E402  (확장을 켠 뒤에 import)

omni.usd.get_context().open_stage(STAGE)
print(f"[open_test2_ros2] 스테이지: {STAGE}")


def build_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                # ROS_DOMAIN_ID 환경변수를 그대로 쓴다. 없으면 domain_id(0).
                ("Context.inputs:useDomainIDEnvVar", True),
                ("PublishClock.inputs:topicName", "clock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    print(f"[open_test2_ros2] 액션그래프 생성: {GRAPH_PATH}  ->  /clock "
          f"(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '미설정')})")


build_graph()

# 스테이지가 완전히 로드된 뒤에 Play 를 누른다. 컨베이어 에셋이 원격(https)
# 참조라 로드가 몇 초 걸린다.
_state = {"frames": 0, "sub": None}
_AUTOPLAY = not os.environ.get("PL_NOPLAY")


def _tick(_event):
    _state["frames"] += 1
    if _state["frames"] < 120:
        return
    if _state["sub"] is None:
        return
    _state["sub"] = None            # 한 번만
    omni.timeline.get_timeline_interface().play()
    print("[open_test2_ros2] 타임라인 시작. 다른 터미널에서 확인:")
    print("    ros2 topic list        # /clock 이 보이면 브리지 정상")
    print("    ros2 topic echo /clock")


if _AUTOPLAY:
    _state["sub"] = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(_tick, name="open_test2_ros2_autoplay")
    )
else:
    print("[open_test2_ros2] PL_NOPLAY=1 — Play 를 직접 눌러야 /clock 이 나온다.")
