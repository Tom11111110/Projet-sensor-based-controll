# Projet-sensor-based-controll
# Robust Nonlinear Controller for a Two-Wheeled Mobile Robot

## Project Overview

This project focuses on the design and implementation of a **robust nonlinear controller** for a two-wheeled mobile robot. The main goal is to make the robot follow a **circular trajectory** while handling external disturbances, ensuring stable and accurate motion.

The project is divided into two main parts:
1. **Software:** Simulations and algorithm development performed on a computer, including Matlab simulations and ROS 2 virtual testing.
   *(For detailed steps and results, see [Software.md](Software.md))*

2. **Hardware:** Implementation on a physical robot (TurtleBot3), including real-time ROS 2 execution, robot control, and demonstration videos.
   *(For detailed steps and results, see [Hardware.md](Hardware.md))*

This structure allows for a smooth transition from simulation to real-world experimentation, ensuring that the controller is robust and reliable before deployment on the actual robot.

# Mathematical Foundations and Control Design

The control design is based on the research presented in the article *"Flatness-based nonsingular fast terminal sliding mode control of Wheeled Mobile Robot with disturbances"*. This approach leverages two key advanced control theories to achieve robust Trajectory Tracking (TT): **Differential Flatness (DF)** and **Robust Nonsingular Fast Terminal Sliding Mode Control (RNFTSMC)**.

## 1. Differential Flatness (DF) for System Simplification

The Wheeled Mobile Robot (WMR) system is initially nonlinear and underactuated:

- **States:** $q = [x, y, \theta]^T$  
- **Inputs:** $v, w$

**Objective:** DF transforms the complex nonlinear WMR model into a simpler, linear canonical form (Brunovsky Form). This simplification streamlines the controller development process.

**Flat Outputs (FOs):**  
The DF property is established by defining the FOs, typically the Cartesian coordinates of the robot's center:

$$
\Omega = [\Omega_{11}, \Omega_{21}]^T = [x, y]^T \quad
$$

**Key Property:** All system states ($x, y, \theta$) and inputs ($v, w$) can be expressed as algebraic functions of the FOs and their derivatives. The linearized system (Brunovsky Form) becomes:

$$
\dot{\Omega}_{11} = \Omega_{12}, \quad \dot{\Omega}_{12} = v_x
$$

$$
\dot{\Omega}_{21} = \Omega_{22}, \quad \dot{\Omega}_{22} = v_y \quad
$$

This effectively converts the trajectory tracking problem into **two independent double-integrator systems** in the $\Omega$ space.

## 2. Robust Nonsingular Fast Terminal Sliding Mode Control (RNFTSMC)

The WMR model, accounting for external perturbations $\rho = [\rho_x, \rho_y]^T$, is given in the linearized form as:

$$
\dot{\Omega}_{12} = v_x + \rho_x, \quad \dot{\Omega}_{22} = v_y + \rho_y \quad 
$$

**Objective:** RNFTSMC is employed to achieve precise tracking and robustness against disturbances and modeling uncertainties.

### Sliding Surface

A robust nonsingular fast terminal sliding surface is defined in terms of the tracking errors $e_1 = x - x_d$ and $e_2 = y - y_d$:

$$
\sigma_x = e_1 + \mu_1 |e_1|^{\Phi_1} \, \mathrm{sgn}(e_1) + \mu_2 |e_3|^{\Theta_1} \, \mathrm{sgn}(e_3) \quad 
$$

$$
\sigma_y = e_2 + \mu_3 |e_2|^{\Phi_2} \, \mathrm{sgn}(e_2) + \mu_4 |e_4|^{\Theta_2} \, \mathrm{sgn}(e_4) \quad 
$$

- Parameters $\mu_i, \Phi_i, \Theta_i$ are chosen such that $1 < \Theta_i < 2$ and $\Phi_i > \Theta_i$ to ensure **Finite-Time Convergence (FTC)** while avoiding the singularity problem of classic Terminal SMC.

### Control Law

The control inputs $v_x$ and $v_y$ are defined as the sum of an equivalent control part ($v_{eq}$) and a switching control part ($v_{sw}$):

$$
v_x = v_{eqx} + v_{swx} \quad 
$$

- The switching component $v_{sw}$ includes terms to actively cancel the effect of bounded perturbations $\rho_i$ and to drive the error onto the sliding surface in finite time.

