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
    prim_path="/World/redCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0]),
)

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/greenCube",
    name="green_cube",
    position=np.array([-1.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 1.0, 0.0]),
)

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/blueCube",
    name="blue_cube",
    position=np.array([1.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/red2Cube",
    name="red2_cube",
    position=np.array([0.0, 1.0, 0.5]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/green2Cube",
    name="green2_cube",
    position=np.array([0.0, 1.0, 1.0]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([0.0, 1.0, 0.0]),
)


world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)
world.reset()

step_count = 0

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1

    if step_count % 100 == 0:
        print(f"Step: {step_count}")
    
simulation_app.close()