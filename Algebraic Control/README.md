# Algebraic Control

This folder contains the MATLAB code used in the experiments and simulations for the IROS article. It computes the required tilting orientation to align the pendulum rod (modeling the first sloshing mode) with the SCARA-type motion of the robot transporting the container.

---

## 📁 Folder Overview

- `Algebraic_control_v2.m`: Main script to compute tilt-based sloshing compensation and optionally export joint trajectories.
- `utils/`: Contains helper functions and trajectory definition blocks.
  - `FIR_traj.m`: FIR filter-based trajectory generator.
  - Simulink block examples for multi-dimensional trajectory generation.
  - Other helper functions

---

## 🚀 Main Script: `Algebraic_control_v2.m`

This script:
- Computes the tilt orientation to reduce sloshing.
- Uses the **COMAU SmartSix** robot model (kinematics, parameters, URDF, etc.).
- Can output a `.csv` file with joint trajectories for real-world execution.
- Depends on trajectory data from FIR filters or manually defined motion profiles.

### Output:
- Tilt-compensated motion profile.
- Optionally: `trajectory.csv` (if export flag is enabled).

---

## 🧰 Trajectory Generation via FIR Filters

Trajectory generation is handled by the `FIR_traj` function, constructed using FIR filters created by:

```matlab
BuildTrajectoryGenerator([0.5 5/Ts 10/Ts^2 60/Ts^3 200/Ts^4], [18.9612], 0.002);
```

### Inputs:
1. **Kinematic constraints**:
   - `[max_disp, max_vel, max_acc, max_jerk, ...]`
2. **Frequencies to suppress** (e.g., dominant sloshing frequency)
3. **Sampling time** `Ts` (e.g., 0.002 s for COMAU at 500 Hz)

### Notes:
- Built using **Simulink** blocks from the Trajectory Toolbox.
- Make sure to save the output of the Symulink Block in a `.mat` file and place it in the folder `utils/Fir_data`
- For multi-dimensional trajectories, assemble multiple 1D blocks (examples in `utils/`).

---

## 📐 Custom Trajectory Inputs (Non-FIR)

To define a trajectory manually without FIR filters, provide the following numeric time-series variables:

- **3D Position + Derivatives**:
  - `rEx, rEdx, rEddx, rEdddx, rEddddx` (X)
  - `rEy, rEdy, rEddy, rEdddy, rEddddy` (Y)
  - `rEz, rEdz, rEddz, rEdddz, rEddddz` (Z)

- **Rotation about Z** (and its derivatives):
  - `th, thd, thdd`

- **Timing Variables**:
  - `Te` — end time
  - `time` — time vector (0:dt:Te)
  - `n` — length of the time vector

Ensure that all variables are consistently sampled and dimensioned.

---

## ⚠️ Important: Center of Rotation

Proper sloshing compensation requires accurately setting the center of rotation. This should align with the **equivalent mass of the pendulum**, typically at:

```
offset_z = (height from flange to tray base)
         + (container height / 2 + hn)
         = (height from flange to tray base) + container height - Ln - ln
```

Make sure to:
- Measure and set this value accurately in the simulation or real-world model.
- Adjust the robot flange-to-container transformation accordingly.

---

## 🧩 Dependencies

- MATLAB R2021a or later
- **Trajectory Toolbox** (for FIR generation)
- Simulink (for FIR block assembly)
- Robotics System Toolbox (for COMAU model)

---

## 📄 Notes

- The current implementation is tailored for the COMAU SmartSix robot.
- Extendable to other robots if kinematics and center-of-rotation are updated.
- Useful for validating orientation-based sloshing suppression.
- The trajectory definition is relative to an initial offset position of the robot, there are not controindications in defining the trajectory in an absolute way. This would simplify the code avoiding useless transformations, so it is recommended for future developments.