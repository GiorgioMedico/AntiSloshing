# Matrix Legacy

This repository contains the full collection of code, models, and experimental resources developed for sloshing-aware trajectory generation, control, and validation of robotic transport systems.

The repository is organized into modular subfolders, each addressing a key aspect of the overall system: from symbolic modeling of liquid dynamics to real-world robotic control and optimization.

---

## 📁 Repository Structure
```
Matrix-Legacy/
├── Algebraic Control/ # Tilt-based sloshing suppression for SCARA-type motions
├── Experiments/ # Scripts and data for experimental validation
├── Kinematics/ # Robot models (COMAU, UR5e) and kinematics utilities
├── Multi-container Non-Prehensile Optimization/ # Trajectory optimization for nonprehensile transport
└── Sloshing Models/ # Symbolic and numerical models of fluid sloshing dynamics
```

---

## 🔧 Subfolder Overview

### 📁 `Algebraic Control/`
Computes tilting orientations to align the pendulum model of sloshing with the direction of motion. Implements FIR-filter-based trajectory generation and sloshing-compensated joint trajectory output for the COMAU SmartSix robot.

> See: [`README.md`](./Algebraic%20Control/README.md)

---

### 📁 `Experiments/`
Contains sanity checks and image processing code. Includes test of trajectory execution on real robots and sloshing height extraction based on video feeds.

> See: [`README.md`](./Experiments/README.md)

---

### 📁 `Kinematics/`
Contains forward/inverse kinematics, Jacobian calculations, and URDF visualization models for COMAU SmartSix and UR5e robots. Includes class-based robot parameter definitions and joint limit utilities.

> See: [`README.md`](./Kinematics/README.md)

---

### 📁 `Multi-container Non-Prehensile Optimization/`
Implements trajectory optimization techniques for transporting multiple cylindrical containers using non-prehensile control.

> See: [`README.md`](./Multi-container%20Non-prehensile%20Optimization/README.md)

---

### 📁 `Sloshing Models/`
Symbolic derivations and numerical simulations of fluid sloshing modeled as pendulum or MSD systems. Includes scripts in MATLAB Live Script format for YX, ZY, and linear pendulum variants, as well as numerical ODE implementations for simulation via `ode45` or Runge-Kutta. Includes `Model Validation` analyses of the results compared to the physical behavior of the liquid, and `Parameter Estimation` of the models' parameters based on Force/Torque sensor readings.

> See: [`README.md`](./Sloshing%20Models/README.md)

---

## 📦 Dependencies

- MATLAB R2024b or later
- Symbolic Math Toolbox (for sloshing model derivations)
- Simulink (for FIR trajectory generation)
- Robotics System Toolbox (for URDFs and kinematic modeling)
- ROS Toolbox (for real-system logging analysis)
- Trajectory Toolbox (for FIR block generation in Algebraic Control)
- Image Processing Toolbox
- Computer Vision Toolbox
- MATLAB Audio Toolbox

---

## 🛠️ Installation
**Clone the repository:**
```bash
git clone https://github.com/sopranisimone/Matrix-Legacy.git
cd Matrix Legacy
```
---

## 📝 Citation

If you use this repository in academic work, please cite the associated articles present in each subfolder.

---

## 📬 Contact

For questions, contributions, or collaboration, please contact the project maintainers.

---
