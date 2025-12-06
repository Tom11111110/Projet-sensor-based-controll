# Projet-sensor-based-controll
# Robust Nonlinear Controller for a Two-Wheeled Mobile Robot

## Project Overview

This project focuses on the design and implementation of a **robust nonlinear controller** for a two-wheeled mobile robot. The main goal is to make the robot follow a **circular trajectory** while handling external disturbances, ensuring stable and accurate motion.

The project is divided into two main parts:  
1. **Software:** Simulations and algorithm development performed on a computer, including Matlab simulations and ROS 2 virtual testing.  
2. **Hardware:** Implementation on a physical robot (TurtleBot3), including real-time ROS 2 execution, robot control, and demonstration videos.

This structure allows for a smooth transition from simulation to real-world experimentation, ensuring that the controller is robust and reliable before deployment on the actual robot.

## Software: Matlab Simulation

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
3. Run the main simulation script:
   ```matlab
   main.m
4. The simulation will:
   - Compute the reference trajectory
   - Apply the RNFTSMC controller
   - Plot the robot’s actual trajectory versus the reference
   - Display the position errors over time
  
### Simulation Results

The simulation shows the robot following a **circular trajectory** while compensating for disturbances.  
The RNFTSMC controller ensures that the robot stays close to the reference path despite external perturbations.

![Circular Trajectory Tracking](matlab_simu_code/results/trajectory_plot.png)

> In the figure above, the **red dashed line** represents the reference circular trajectory,  
> while the **blue line** shows the actual trajectory of the robot.  
> The starting position is marked with a red circle.  
> The results demonstrate the effectiveness of the robust nonlinear controller in maintaining accurate tracking.

