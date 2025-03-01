import asyncio
import numpy as np
import omni
from pxr import UsdPhysics
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.world import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage
import time

async def example():
    init_time = time.time()
    if World.instance():
        World.instance().clear_instance()
    world=World()
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()

    # add franka articulations
    asset_path = "/isaac-sim/NewSimulation/sample-ackermann-amr/assets/F1Tenth.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/RC_CAR")
    # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")

    # batch process articulations via an ArticulationView
    frankas_view = ArticulationView(prim_paths_expr="/RC_CAR", name="CAR")
    world.scene.add(frankas_view)
    await world.reset_async()

    omni.timeline.get_timeline_interface().play()
    dc = _dynamic_control.acquire_dynamic_control_interface()
    articulation = dc.get_articulation("/RC_CAR")
    while time.time()-init_time< 500:
        for i in range(34):
            dc.wake_up_articulation(articulation)

            # position control
            #joint_angles = [np.random.rand(9) * 2 - 1]
            joint_angles = 34 * [0]
            joint_angles[i] = 1#np.random.rand(1) * 2 -1
            dc.set_articulation_dof_position_targets(articulation, joint_angles)
            
            await asyncio.sleep(5)
        # single DOF position Control
        #dof_ptr = dc.find_articulation_dof(articulation, "Steering_Knuckle_Right")
        #dof_ptl = dc.find_articulation_dof(articulation, "Steering_Knuckle_Left")
        #dc.set_dof_position_target(dof_ptr, -1.5)
        #dc.set_dof_position_target(dof_ptl, -1.5)

        # 


        await asyncio.sleep(1)
    

asyncio.ensure_future(example())