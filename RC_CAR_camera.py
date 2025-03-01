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

    camera_l = Camera(prim_path = '/RC_CAR/Rigid_Bodies/Chassis/Camera_Left')
    camera_l.initialize()

    camera_r = Camera(prim_path = '/RC_CAR/Rigid_Bodies/Chassis/Camera_Right')
    camera_r.initialize()
    
    camera_l.add_motion_vectors_to_frame()
    camera_l.add_distance_to_image_plane_to_frame()


    while time.time()-init_time< 25:
        dc.wake_up_articulation(articulation)

        # position control
        #joint_angles = [np.random.rand(9) * 2 - 1]
        joint_angles = 34 * [0]
        cmd = cmd + sign * intervals
        if cmd>=1:
            sign = -1
        elif cmd<= -1:
            sign = 1
        joint_angles[19] = cmd # right wheel joint
        joint_angles[22] = cmd # left wheel joint
        #dc.set_articulation_dof_position_targets(articulation, joint_angles)
        
        #await asyncio.sleep(0.002)
        # single DOF position Control
        dof_ptr = dc.find_articulation_dof(articulation, "Knuckle__Upright__Front_Right")
        dof_ptl = dc.find_articulation_dof(articulation, "Knuckle__Upright__Front_Left")
        dc.set_dof_position_target(dof_ptr, cmd)
        dc.set_dof_position_target(dof_ptl, cmd)
        frame = camera_l.get_rgb()
        #print(camera_l.get_())
        depth = camera_l.get_depth()
        #inf_mask = np.isinf(depth)
        #print(isinstance(depth, np.ndarray))
        #depth[inf_mask] = 3000
        if depth is not None:
            inf_mask = np.isinf(depth)
            depth[inf_mask] = 3000
            print(depth)
        #print(depth[0])
        #plt.imshow(camera_l.get_rgba()[:,:,:3])
        #plt.show()
        await asyncio.sleep(0.002)

    

asyncio.ensure_future(example())


