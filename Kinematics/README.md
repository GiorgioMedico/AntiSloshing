# Kinematics

This folder contains the kinematic models, utility functions, and URDF files for robotic platforms used in the anti-sloshing project. It currently supports:

- **COMAU SmartSix**
- **UR5e (Universal Robots)**

The folder includes both symbolic/numerical kinematics functions and visualization resources for simulation and analysis.

## 🤖 COMAU SmartSix

### Provided Functions:

- `SmartSix.m`: class for a Comau SmartSix object instance with DH parameters, joint limits and methods for kinematics.
- `create_SmartSix.m`: class for a Comau SmartSix object instance with DH parametersa and joint limits.
- `SmartSix_FK.m`: Computes the forward kinematics from joint values to end-effector pose.
- `SmartSix_IK.m`: Solves the inverse kinematics to compute joint values from a desired pose, selecting as parameter to the function the configuration.
- `SmartSix_Jg.m`: Calculates the geometric Jacobian matrix for a given joint configuration.
- `SmartSIX_fk_j.m`: Script to define the Matlab Functions `SmartSix_FK.m` and `SmartSix_Jg.m`.

Note: the `_sym` function variations are defined in order to be used inside the CasADI framework.

### Subfolder:
- `URDF`: containes the URDF file of the system, useful in the visualization in Matlab through the **Robot System Toolbox**

---

## 🤖 UR5e

- No URDF is given as the robot is already part of the standard **Robot System Toolbox** visual meshes. 
- Kinematic functions implemented as `UR_FK`, `Inv_Pos_UR` and `UR_Jg`
- `create_robot.m` creates the UR instance.

---

## Dependencies

- MATLAB R2021a or later (for full symbolic support and Live Scripts)
- **ROS Toolbox** (for rosbag import)
- MATLAB Robotics System Toolbox (`importrobot`)


