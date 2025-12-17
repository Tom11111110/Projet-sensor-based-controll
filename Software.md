# Software: 
### Matlab Simulation
### Overview
The Matlab code implements a **Robust Nonlinear Fast Terminal Sliding Mode Controller (RNFTSMC)** for a two-wheeled mobile robot.  
The main goal is to make the robot follow a **circular trajectory** while handling external disturbances.

All Matlab scripts are located in the folder: `matlab_simu_code/`

### How to Run
1. Download all files from the repository and place them in the same folder on your computer.  
   - Make sure `main.m`, `trajectory_reference.m`, `error_dynamics_RNFTSMC.m`, and any other required scripts are **in the same directory**.
2. Open Matlab and navigate to that folder:
   ```matlab
   cd 'path_to/matlab_simu_code'
    ```
3. Run the main simulation script:
   ```matlab
   main.m
4. The simulation will:
   - Compute the reference trajectory
   - Apply the RNFTSMC controller
   - Plot the robot’s actual trajectory versus the reference
   - Display the position errors over time
  
### Simulation Results

#### 1. Robot vs Reference Trajectory
<p align="center">
  <img src="Matlab_simu_code/results/trajectory_plot.png" alt="Robot vs Reference Trajectory" width="900">
</p>

**Figure:** The blue line shows the robot’s actual path while the red dashed line shows the circular reference trajectory.

#### 2. Position Errors
<p align="center">
  <img src="Matlab_simu_code/results/position_errors.png" alt="Position Errors" width="900">
</p>

**Figure:** Errors in X and Y positions over time. The controller reduces errors despite disturbances.

#### 3. Sliding Surfaces
<p align="center">
  <img src="Matlab_simu_code/results/sliding_surfaces.png" alt="Sliding Surfaces" width="900">
</p>

**Figure:** Sliding surfaces σ_x and σ_y vs time, showing the convergence properties of the RNFTSMC controller.

#### 4. Orientation vs Reference
<p align="center">
  <img src="Matlab_simu_code/results/orientation_vs_reference.png" alt="Orientation vs Reference" width="900">
</p>

**Figure:** Robot orientation θ (green) vs reference θ_ref (red dashed) over time.

#### 5. Control Inputs vs Time
<p align="center">
  <img src="Matlab_simu_code/results/control_inputs.png" alt="Control Inputs vs Time" width="900">
</p>

**Figure:** The control signals v_x (blue) and v_y (red) applied to the robot over time.

#### 6. Moment of Disturbance (Optional)
<p align="center">
  <img src="Matlab_simu_code/results/disturbance_moment.png" alt="Disturbance vs Time" width="900">
</p>

**Figure:** Shows when the external disturbance is applied (between t_start and t_end). This helps visualize its effect on robot trajectory and control signals.



## ROS 2 / WSL Simulation Tutorial

The ROS 2 simulation provides a realistic environment to test the controller before deploying it on physical hardware (TurtleBot3). We use the RNFTSMC Controller package developed in Python (`simu_rnftsmc`).

## 1. Prerequisites and Environment Setup

This tutorial assumes you are working either within **WSL 2** or a **native Linux installation** (Ubuntu), and that you have a functional **ROS 2 Humble** (or later) setup.

### Dependencies
Ensure you have the necessary ROS 2 packages and Python dependencies installed.

```bash
# Update ROS 2 dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### Navigate to the Workspace Root
The ROS 2 workspace root is where the `src` folder is located (inside `Ros_code`).

```bash
cd ~/Projet-sensor-based-controll/Ros_code
```

## 2. Building the ROS 2 Controller Package

You need to build the Python package (`simu_rnftsmc`) so that the executable node is created and recognized by ROS 2.

### Build the package
```bash
colcon build --packages-select simu_rnftsmc
```

### Source the environment
This step is crucial and must be done in every new terminal you open before running ROS 2 commands.

```bash
source install/setup.bash
```

> **Note:** If you are using a base ROS 2 setup, you might also need to source your main ROS installation:
> ```bash
> source /opt/ros/humble/setup.bash
> ```

## 3. Running the Simulation Node

The simulation is a simple self-contained environment where the controller node calculates the dynamics, updates the robot's pose, and publishes the results.

### Execute the RNFTSMC Controller Node
```bash
ros2 run simu_rnftsmc rnftsmc_controller
```

The node will start running at the defined control rate (50 Hz by default). You should see the initial log message:
```
RNFTSMC controller started at 50.0 Hz.
```

## 4. Visualization with RViz2

To visualize the trajectory, robot pose, and the path, you need to launch RViz2 in a separate terminal.

### Open a New Terminal (WSL)
Source the environment (mandatory):

```bash
cd ~/Projet-sensor-based-controll/Ros_code
source install/setup.bash
```

### Launch RViz2
```bash
rviz2
```

### Configure RViz2
Once the RViz window opens:
1. In the **Display** panel, set the **Fixed Frame** to `map`.
2. Click **Add** (bottom left) and add the following displays:
   - **Marker** (Topic: `/reference_trajectory`) → Shows the red circular path.
   - **Path** (Topic: `/robot_path`) → Shows the blue line traced by the robot.
   - **Pose** (Topic: `/pose`) → Shows the current estimated pose/position.

You will see the robot's pose (`base_link`) closely follow the red reference circle, demonstrating the controller's effectiveness in tracking and disturbance rejection, mirroring the results seen in the Matlab simulation.

## Simulation Result

Below is an example of the trajectory tracking obtained in RViz2:

<p align="center">
  <img src="Ros_result/Rviz_simu.png" alt="RViz2 simulation result" width="900">
</p>

**Figure:** The robot’s pose (`base_link`) follows the red circular reference trajectory.

