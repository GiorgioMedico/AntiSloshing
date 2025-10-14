# Antisloshing Non-Prehensile Trajectory Optimization

## Table of Contents
* [Overview](#overview)
* [Theoretical Background](#theoretical-background)
* [Repository Structure](#repository-structure)
    * [Directories](#directories)
    * [Main Files](#main-files)
* [Getting Started](#getting-started)
    * [Prerequisites](#prerequisites)
    * [Environment Setup](#environment-setup)
    * [Running the Code](#running-the-code)
* [Configuration and Customization](#configuration-and-customization)
* [Troubleshooting](#troubleshooting)
* [Author Information](#author-information)

## Overview
This repository contains the code and data for the time-optimal trajectory planning of multi-container non-prehensile transport using a robotic manipulator. The developed algorithms investigate two primary scenarios:
1.  **Liquid-filled Containers (Anti-Sloshing Purpose):** Addresses the "waiter-motion problem" where containers are partially filled with liquid, requiring consideration and minimization of liquid sloshing.
2.  **Rigid Body Containers (No Anti-Sloshing Purpose):** Focuses solely on the "waiter-motion problem" for solid objects, optimizing the robot's motion to prevent slipping, tipping, and rotation of the containers on the tray.

The core objective in both cases is to determine time-optimal trajectories that adhere to a set of physical constraints while ensuring stable and efficient transport.

## Theoretical Background

This project builds upon principles of robotics, optimal control, and fluid dynamics:

* **Waiter Motion Problem:** A problem in robotics focusing on generating trajectories for a robot carrying objects on a tray without grasping them. The key challenges are preventing the objects from slipping, tipping over, or detaching from the tray due to inertial forces during acceleration and deceleration. This involves friction, tipping, and detachment constraints.
* **Liquid Sloshing Dynamics:** For liquid-filled containers, the motion of the liquid inside can cause undesired sloshing, potentially leading to spillage. This project utilizes a **mass-spring-damper model** to accurately represent the liquid's dynamic behavior and incorporate sloshing height limits into the optimization problem.
* **Optimal Control:** The problem is formulated as a non-linear optimal control problem (OCP) where the goal is to minimize a cost function subject to dynamic equations (trajectory and liquid motion) and inequality constraints (e.g., sloshing limits, non-prehensile conditions, robot joint limits).
* **Non-linear Programming (NLP):** The OCP is discretized using a multiple shooting method, transforming it into a non-linear programming problem, which is then solved numerically using IPOPT.

## Repository Structure

```
.
├── casadi-3.6.5-windows64-matlab2018b/
├── Data/
├── include/
├── Plots/
├── NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m
└── NonPrehensile_Assigned_Path_Optimization.m
```

### Directories:

* **`casadi-3.6.5-windows64-matlab2018b/`**:
    This directory contains the necessary CasADi libraries (version 3.6.5 for Windows 64-bit with MATLAB 2018b compatibility).

* **`Data/`**:
    This directory serves as the storage for all generated output data files.
    * `.mat` files: These files store results from optimization simulations, including time-series data for sloshing heights.
    * `.csv` trajectory files: These files contain the joint trajectory data, representing the optimized motion profiles for non-prehensile manipulation.

* **`include/`**:
    This directory houses the MATLAB functions that are utilized across the main scripts. These functions encapsulate common calculations, model definitions, or utility operations.

* **`Plots/`**:
    This directory is designated for saving all generated figures derived from the simulations.

### Main Files:

* **`NonPrehensile_Assigned_Path_Optimization.m`**:
    This MATLAB script is the core of the optimal control problem for **anti-sloshing non-prehensile manipulation** involving multiple liquid-filled containers.
    * **Functionality**: This script defines and solves the optimization problem. It begins by importing the robot model and configuring the problem's setup, including debug flags, container and liquid parameters and disposition, initial conditions and path definition. The optimal control problem is then formulated, incorporating constraints related to the "waiter problem" (non-slipping, non-tipping, non-lifting, non-torsional rotation) and, the imposed sloshing height limits for each container, modeled using a mass-spring-damper equivalent system.
    * **Setup**: The robot simulated model is defined using the Robotics System Toolbox, which allows for the visualization of the robot behavior, and also an object representation of the robot's physical structure and kinematics is defined. The setup is also configured to include the robot's end-effector pose relative to the tray, the tray dimensions, and the configuration of the containers.
    * **Path and Control**: The geometric path the tray must follow is **prescribed** and parameterized by a scalar coordinate $s$, which can be chosen among a list of preset ones or defined by the user during execution. The rotation about the z-axis of the tray is also expressed as a linear function of $s$. The primary control input to the optimization problem is the jerk of $s$ ($\dddot{s}$).
    * **Optimization Objective**: The primary objective is to determine an optimal trajectory that minimizes the total time taken to traverse the path ($t_e$), while also regularizing the control input (jerk of $s$) to ensure a smooth motion.
    * **Dynamics**: The dynamics of the system state, which include the path parameter $s$ and its derivatives, as well as the position and velocities of the masses within the MSD (Mass-Spring-Damper) sloshing models, are solved numerically using a Runge-Kutta integration scheme. The MSD dynamics specifically follow the Equations of Motion (EOMs) defined for a 4D MSD model, accounting for both translational and rotational accelerations of the container.
    * **Constraints**: The problem includes constraints for sloshing height limits, robot joint limits, and the aforementioned "waiter problem" constraints. The sloshing height is modeled using a mass-spring-damper system, which allows for the prediction of liquid behavior during the robot's motion.
    * **Solver**: The problem is solved by using the IPOPT solver within the CasADi framework.
    * **Output**: Upon completion, the script returns figures, `.mat` files, and `.csv` files, which detail the optimized trajectory, sloshing behavior, and robot states.

* **`NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m`**:
    This MATLAB script is another core optimization file, specifically focusing on scenarios where **liquid sloshing is not considered**, i.e., containers are treated as rigid bodies.
    * **Functionality**: This script closely resembles `NonPrehensile_Assigned_Path_Optimization.m` in its overall structure and problem-solving approach. However, a key distinction is that liquid slosh dynamics are *not* included in this optimization problem. Therefore, the only active physical constraints, apart from the robot's own limitations, are those related to the "waiter problem" (preventing relative motion such as slipping, tipping, detachment, and torsional rotation with respect to the tray).
    * **Enhanced Degrees of Freedom**: Crucially, this script leverages additional degrees of freedom of the robot. While the geometric path is still prescribed by $s$ and the yaw angle $\theta_z$ is also prescribed (typically as the first rotation in an Euler sequence to define the "absolute" orientation of the tray), the **other two Euler angles ($\theta_x$ and $\theta_y$, representing pitch and roll)** are **free to be optimized**. This allows the robot to dynamically tilt the tray, exploiting the forces acting on the containers to keep them securely on the tray with increased performance.
    * **Control Input**: The control input for this optimization problem is the vector of jerks: $\textbf{u}(t) = [\;\dddot{s}(t) \;\; \dddot{\theta}_x(t) \;\; \dddot{\theta}_y(t)\;]^T$.
    * **Optimization Objective**: The problem is solved using IPOPT within the CasADi framework, aiming to find an optimal trajectory that minimizes total time ($t_e$) and regularizes the control input. 
    * **Output**: Similar to the sloshing-specific script, this file offers options to save optimized trajectories and results for analysis.

## Getting Started

### Prerequisites
To run the code in this repository, you will need the following software installed:

* **MATLAB**: Version 2023b or compatible. Ensure you have a valid license.
* **CasADi library**: The necessary CasADi binaries for MATLAB are provided within the `casadi-3.6.5-windows64-matlab2018b/` directory. No separate installation is typically required beyond ensuring this directory is in your MATLAB path.
* **Robotics System Toolbox**: This MATLAB toolbox is required for robot model definition and visualization.

### Environment Setup
1.  **Clone the Repository:** Download or clone this repository to your local machine.

### Running the Code
1.  **Open MATLAB:** Launch your MATLAB environment.
2.  **Navigate to Project Directory:** In the MATLAB command window, navigate to the root directory of this repository.
3.  **Run Desired Script:** Execute the desired main script from the MATLAB command window or editor:
    * For anti-sloshing optimization: `NonPrehensile_Assigned_Path_Optimization.m`
    * For rigid body optimization: `NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m`
4.  **Follow the Interactive Workflow**: Upon running an optimization script (`NonPrehensile_Assigned_Path_Optimization.m` or `NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m`), a series of interactive GUI windows will appear to guide you through the problem setup.

    * **Window 1: Options**:
        * Choose which results to save: Figures, CSV data (optimized joint angles trajectories), MAT files containing sloshing height simulation results.
        * Decide whether to show animations during code execution.
        * Decide on a CSV joint offset if needed for a specific trajectory.

    * **Window 2: Container and Liquid Properties**:
        * Input geometric parameters for the container (e.g., radius, height, filling height for liquid cases).
        * Specify measured properties like container mass and center of mass height.
        * For liquid-filled containers, input liquid properties such as density and viscosity and maximum allowable sloshing height.

    * **Window 3: Container Placement Configuration**:
        * Specify the total number of containers being transported.
        * Define their initial positions on the tray by choosing from a set of predefined locations or by manually entering coordinates.
        * Choose between predefined tray configurations or input custom values for the tray dimensions and the friction coefficient between the containers and the tray surface.

    * **Window 4: Tray-to-End-Effector Pose**:
        * Define the fixed relative transformation (position and orientation) between the tray and the robot's end-effector. This links the tray's motion to the robot's kinematics. Notably this script allows only for an offset rotation around the vertical axis but expert users can modify the code to include more complex transformations if needed.

    * **Window 5: Tray Motion Parameters**:
        * Set the prescribed geometric motion path (e.g., a straight line, an arc) or define a custom one.
        * Choose whether the trajectory definition is relative or absolute. If relative, define the specific point from which the trajectory is calculated.
        * Specify the initial and final orientation of the tray (yaw angle).
5.  **View the Results**: After the setup is complete and the optimization or validation script has finished execution, numerous figures will be displayed showing the optimized trajectories, compliance with constraints, and other useful information.

## Configuration and Customization

Beyond the interactive GUI windows, advanced users can customize the optimization problem and model parameters by directly modifying the `.m` scripts within the `include/` directory and the main optimization files. This includes:

* **Weighting Factors**: Adjust `k`, etc., in the cost function to prioritize time minimization versus trajectory smoothness for specific applications.
* **Robotic System**: Update or change the robot model definition if a different manipulator is to be used by changing the `SmartSix()` instantiation in the main scripts and by updating the `URDF` file imported. Importantly, ensure that the new class definition in the `include/` directory matches the `SmartSix()` class structure and that the appropriate methods for kinematics calculations are implemented (following the existing patterns).
* **Path Definition**: Define more complex or custom prescribed motion paths in the `Trajectory` sections of the code. If the spline interpolation is still desired, define the number of control points and their positions as vectors, in variables `cpx, cpy, cpz, n_ctrl_pts`. If more complex or different paths are needed, consider implementing a custom path definition function and modifying the `Trajectory` section in the `include/FormulateTrajectory.m` script to suit your needs.

## Troubleshooting

* **"Undefined function or variable 'casadi'"**: Ensure the `casadi-3.6.5-windows64-matlab2018b/` directory and its contents are correctly added to your MATLAB path. Restart MATLAB if necessary.
* **Optimization Fails to Converge**: This can be due to overly restrictive constraints, poor initial guesses for the optimization variables, or numerical instability. Try relaxing constraints slightly, providing a more reasonable initial trajectory, or adjusting solver parameters (e.g., IPOPT options).
* **Robotics System Toolbox Errors**: Verify that the Robotics System Toolbox is installed and licensed correctly within your MATLAB installation.
* **"Out of Memory" Errors**: For very long trajectories or a large number of containers, the optimization problem can become very large. Consider reducing the number of multiple shooting intervals or simplifying the models if possible.
* **NaN or Inf Values in Results**: This may indicate numerical instability during optimization. Check the initial conditions, constraints, and ensure that the constraints are differentiable.

## Author Information

This project was developed by S. Soprani (simone.soprani2@unibo.it) as part of the MATRIX Project.
