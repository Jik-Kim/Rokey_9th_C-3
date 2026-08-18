from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

step_count = 0

while simulation_app.is_running():
  # 1. GUI에서 Stop 버튼을 누른 정지 상태이면 step_count를 0으로 리셋
    if world.is_stopped():
        step_count = 0
    world.step(render=True)

  # 2. Play 버튼이 눌려 시뮬레이션이 재생 중일 때만 step_count 증가 및 출력
    if world.is_playing():
        time.sleep(0.01)
        step_count += 1

        if step_count == 1:
            print("[리셋] Play 시작 -> stop_count = 0")

        if step_count % 100 == 0:
            print(f"Step: {step_count}")

        if step_count == 300:
            cube_prim.set_world_pose(position=np.array([0.0, 0.0, 1.0]))
            cube_prim.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
            print("[이동] 큐브 순간이동")

simulation_app.close()