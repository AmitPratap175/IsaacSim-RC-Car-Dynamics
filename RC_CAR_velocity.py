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
    stage = omni.usd.get_context().get_stage()

    # for prim in stage.TraverseAll():
    #     prim_type = prim.GetTypeName()
    #     if prim_type == "PhysicsRevoluteJoint":
    #         drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    #         if drive:
    #             print(f"Setting stiffness for: {prim.GetPath()}")
    #             drive.GetStiffnessAttr().Set(0)
    #         else:
    #             print(f"Drive API not found for: {prim.GetPath()}")

    #     else:
    #         drive = UsdPhysics.DriveAPI.Get(prim, "linear")
    
    joint1_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Knuckle__Front_Right"), "angular")
    joint2_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Knuckle__Front_Left"), "angular")
    joint3_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Upright__Rear_Right"), "angular")
    joint4_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Upright__Rear_Left"), "angular")

    # joint2_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Knuckle__Front_Left"), "angular")
    # joint1_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/RC_CAR/Joints/Wheel__Knuckle__Front_Right"), "angular")

    print('\n\n\nTrue........',joint1_drive)
    print('True........',joint2_drive)
    print('True........',joint3_drive)
    print('True........',joint4_drive)
    joint1_drive.GetStiffnessAttr().Set(0)
    joint2_drive.GetStiffnessAttr().Set(0)
    joint3_drive.GetStiffnessAttr().Set(0)
    joint4_drive.GetStiffnessAttr().Set(0)
    dc = _dynamic_control.acquire_dynamic_control_interface()
    articulation = dc.get_articulation("/RC_CAR")



    while time.time()-init_time< 25:
        dc.wake_up_articulation(articulation)
        # joint_vels = [-np.random.rand(34)*10]
        # dc.set_articulation_dof_velocity_targets(articulation, joint_vels)
        dof_ptr1 = dc.find_articulation_dof(articulation, "Wheel__Knuckle__Front_Left")
        dof_ptr2 = dc.find_articulation_dof(articulation, "Wheel__Knuckle__Front_Right")
        dof_ptr3 = dc.find_articulation_dof(articulation, "Wheel__Knuckle__Rear_Left")
        dof_ptr4 = dc.find_articulation_dof(articulation, "Wheel__Knuckle__Rear_Right")
        dc.set_dof_velocity_target(dof_ptr1, 0.2)
        dc.set_dof_velocity_target(dof_ptr2, 0.2)
        dc.set_dof_velocity_target(dof_ptr3, 0.2)
        dc.set_dof_velocity_target(dof_ptr4, 0.2)
        await asyncio.sleep(0.002)

    

asyncio.ensure_future(example())

