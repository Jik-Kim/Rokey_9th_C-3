# isaac-sim.sh --exec 로 실행되어 test1.usd 를 바로 연다.
#   cd ~/isaacsim && ./isaac-sim.sh --exec ~/cobot3_ws/isaacpjt/M0609/hak/project/open_test1.py
import omni.usd

STAGE = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
omni.usd.get_context().open_stage(STAGE)
