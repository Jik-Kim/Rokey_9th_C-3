from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim_b = DynamicCuboid(                              # 4. Prim
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([0.0, 0.0, 1.0]),
)
world.scene.add_default_ground_plane()                  # 5. Scene

world.scene.add(cube_prim_b)

world.reset()
step_count = 0
was_playing = False

while simulation_app.is_running():                      # 6. Simulation
    
    world.step(render=True)
    time.sleep(0.01)

    is_playing = world.is_playing()

    # Stop -> Play로 전환된 순간 감지
    if is_playing and not was_playing:
        step_count = 0
        print(f"[리셋] Play 시작 -> step_count = {step_count}")

    was_playing = is_playing

    if is_playing:
        step_count += 1

        if step_count % 100 == 0:
            print(f"step: {step_count}")

        if step_count % 300 == 0:
            # 랜덤 위치로 이동 (원하는 좌표로 바꿔도 됨)
            print("[이동] 큐브 순간이동")
            new_pos = np.array([0.0, 0.0, 1.0])
            cube_prim_b.set_world_pose(position=new_pos)

simulation_app.close()