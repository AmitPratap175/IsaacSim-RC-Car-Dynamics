import asyncio
import numpy as np
import omni
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.world import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import time


class CAR:
    """
    A class to manage the Isaac Sim environment.
    """
    def __init__(self):
        self.world = None

    async def initialize(self):
        """
        Initializes the Isaac Sim environment.
        """
        if World.instance():
            World.instance().clear_instance()
        self.world = World()
        await self.world.initialize_simulation_context_async()
        self.world.scene.add_default_ground_plane()
        self.add_car()
        await self.create_articulation_view()


    def add_car(self, asset_path = "/isaac-sim/NewSimulation/sample-ackermann-amr/assets/F1Tenth.usd"):
        """
        Adds an Ackermann steered robot robot (or any USD asset) to the simulation.
        """
        add_reference_to_stage(usd_path=asset_path, prim_path="/RC_CAR")

    async def create_articulation_view(self, prim_paths_expr = "/RC_CAR", name = "CAR"):
        """
        Creates an ArticulationView for easier access to articulations.
        """
        car_view = ArticulationView(prim_paths_expr=prim_paths_expr, name=name)
        self.world.scene.add(car_view)
        await self.world.reset_async()

    def play(self):
        """
        Starts the simulation timeline.
        """
        omni.timeline.get_timeline_interface().play()


class CarController:
    """
    A class to control the robot in the simulation.
    """
    def __init__(self, articulation_path="/RC_CAR"):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.articulation = self.dc.get_articulation(articulation_path)
        self.sign = 1
        self.intervals = 0.01
        self.cmd = 0

    def set_joint_angles(self, joint_angles):
        """
        Sets the joint angles for the robot.
        """
        self.dc.wake_up_articulation(self.articulation)
        self.dc.set_articulation_dof_position_targets(self.articulation, joint_angles)

    def steering(self):
        """
        Controls a single degree of freedom (DOF) of the robot.
        remember index is starts from 0 
        joint_angles index 19: right wheel joint
        joint_angles index 22: left wheel joint
        """
        self.update_control()
        self.dc.wake_up_articulation(self.articulation)
        dof_ptr = self.dc.find_articulation_dof(self.articulation, "Knuckle__Upright__Front_Right")
        dof_ptl = self.dc.find_articulation_dof(self.articulation, "Knuckle__Upright__Front_Left")
        self.dc.set_dof_position_target(dof_ptr, self.cmd)
        self.dc.set_dof_position_target(dof_ptl, self.cmd)
        

    def update_control(self):
        """
        Updates the control signal for the robot.
        """
        self.cmd += self.sign * self.intervals
        if self.cmd >= 1:
            self.sign = -1
        elif self.cmd <= -1:
            self.sign = 1


class RobotCamera:
    """
    A class to access and display camera data from the robot.
    """
    def __init__(self, camera_path):
        self.camera = Camera(prim_path=camera_path)
        self.camera.initialize()
        self.camera.add_motion_vectors_to_frame()
        self.camera.add_distance_to_image_plane_to_frame()

    def get_rgb(self):
        """
        Returns the RGB image captured by the camera.
        """
        return self.camera.get_rgb()

    def get_depth(self):
        """
        Returns the depth image captured by the camera.
        """
        depth = self.camera.get_depth()
        if depth is not None:
            inf_mask = np.isinf(depth)
            depth[inf_mask] = 3000
        #while depth is None:
        #    depth = await self.camera.get_depth()
        return depth

    def get_rgba(self):
        """
        Returns the RGBA image captured by the camera (including alpha channel).
        """
        return self.camera.get_rgba()
    
async def main():
    car = CAR()
    init_time = time.time()
    await car.initialize()
    car.play()

    controller = CarController()

    left_camera = RobotCamera('/RC_CAR/Rigid_Bodies/Chassis/Camera_Left')
    
    # try:
    while time.time() - init_time < 20:
        #print('11111111111111')
        controller.steering()
        #print('22222222222222222222')
        print(left_camera.get_depth())
        await asyncio.sleep(0.002)
    # except Exception as err:
    #     print(f'Exception: {err}')



if __name__ == "__main__":
    asyncio.ensure_future(main())
