# IsaacSim-RC-Car-Dynamics

This project demonstrates a simulation of an RC car using NVIDIA Isaac Sim. The simulation includes the setup of the RC car's articulation, control of its wheel joints, and visualization of its movement in a 3D environment.

## Features

### 1. **RC Car Articulation**
   - The RC car is modeled as an articulation, which allows for complex joint movements and interactions.
   - The car's wheels are controlled using `UsdPhysics.DriveAPI` to set the stiffness and velocity targets for each wheel joint.

### 2. **Dynamic Control**
   - The simulation uses the `_dynamic_control` interface to manage the articulation's degrees of freedom (DOFs).
   - The wheels' velocity targets are set dynamically to simulate the car's movement.

### 3. **Simulation Environment**
   - The simulation is set up in a 3D world with a default ground plane.
   - The RC car is added to the scene using a USD file (`F1Tenth.usd`), which contains the car's model and joint configurations.

### 4. **Asynchronous Simulation**
   - The simulation runs asynchronously using Python's `asyncio` library, allowing for smooth and continuous updates to the car's state.
   - The simulation loop runs for a fixed duration (25 seconds), during which the car's wheel velocities are continuously updated.

### 5. **Joint Control**
   - The stiffness of the wheel joints is set to `0` to allow for free movement.
   - The velocity targets for the front and rear wheels are set to `0.2`, simulating the car's forward motion.

### 6. **Visualization**
   - The simulation can be visualized in real-time using NVIDIA Isaac Sim's rendering capabilities.
   - The car's movement and joint behavior can be observed in the 3D environment.

## Code Overview

### Key Components

- **World Initialization**: The simulation world is initialized, and a ground plane is added to the scene.
- **Articulation Setup**: The RC car is added to the scene using a USD file, and an `ArticulationView` is created to manage the car's joints.
- **Joint Configuration**: The stiffness of the wheel joints is set to `0`, and velocity targets are applied to simulate movement.
- **Simulation Loop**: The simulation runs for 25 seconds, during which the car's wheel velocities are continuously updated.

### Key Functions

- `save_rgb(rgb_data, file_name)`: Saves an RGB image to a file.
- `example()`: The main asynchronous function that sets up and runs the simulation.

## Usage

To run the simulation, simply execute the script in an environment where NVIDIA Isaac Sim is installed and configured. The simulation will start automatically, and the RC car will begin moving forward.

```bash
python rc_car_simulation.py
```

## Dependencies

- NVIDIA Isaac Sim
- Python 3.7+
- `numpy`
- `asyncio`
- `PIL` (Pillow) for image saving (optional)

## Notes

- Ensure that the USD file (`F1Tenth.usd`) is correctly referenced in the script.
- The simulation duration can be adjusted by modifying the `while` loop condition in the `example()` function.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

This README provides an overview of the RC car simulation project, its features, and how to run it. For more detailed information on setting up NVIDIA Isaac Sim, refer to the official documentation.