import asyncio
import numpy as np
import omni
import os
from pxr import UsdPhysics
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.world import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import matplotlib.pyplot as plt
from PIL import Image
# from pxr import Sdf, UsdGeom
import time

NUM_FRAMES = 5

# Save rgb image to file

def save_rgb(rgb_data, file_name):
    rgb_img = Image.fromarray(rgb_data, "RGBA")
    rgb_img.save(file_name + ".png")

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

    # batch process articulations via an ArticulationView
    frankas_view = ArticulationView(prim_paths_expr="/RC_CAR", name="CAR")
    world.scene.add(frankas_view)
    await world.reset_async()

    omni.timeline.get_timeline_interface().play()
    dc = _dynamic_control.acquire_dynamic_control_interface()
    articulation = dc.get_articulation("/RC_CAR")
    sign = 1
    intervals = 0.01
    cmd = 0

    # Get information about the structure of the articulation
    num_joints = dc.get_articulation_joint_count(articulation)
    num_dofs = dc.get_articulation_dof_count(articulation)
    num_bodies = dc.get_articulation_body_count(articulation)

    print(f'Number of Joints: {num_joints}')
    print(f'Number of DOFs: {num_dofs}')
    print(f'Number of bodies: {num_bodies}')

    # print the joint states
    dof_states = dc.get_articulation_dof_states(articulation, _dynamic_control.STATE_ALL)
    # print(dof_states)



    

asyncio.ensure_future(example())



