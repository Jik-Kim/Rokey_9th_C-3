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
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

step_count = 0
initial_position = np.array([0.0, 0.0, 0.5])  # Store the initial position of the cube

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1

    if step_count % 100 == 0:
        print(f"Step: {step_count}")

    if step_count == 300 :
        cube_prim.set_world_pose(position=initial_position)
        cube_prim.set_linear_velocity(np.array([0.0, 0.0, 0.0]))

        print("[이동] 큐브 순간이동")

    if step_count == 500:
        print("500스텝 달성! 시뮬레이션을 종료합니다.")
        break

simulation_app.close()
