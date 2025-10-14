# Sloshing Models

This folder contains the implementation of the sloshing dynamics used in the anti-sloshing control project. It includes both the symbolic derivation of the equations of motion and their numerical implementation for simulation. The equivalent dynamical models of MSD and PEN are defined in this folder. Additionaly the scripts used to validate the models and to estimate their parameters are present. 

## Folder Structure
```
Sloshing Models/
├── mlx/ # Symbolic derivations using MATLAB Live Scripts
│ ├── YX/ # Spherical pendulum with YX parametrization
│ ├── ZY/ # Spherical pendulum with ZY parametrization
│ ├── linear/ # Linear pendulum model
│ └── MSD/ # Mass-spring-damper model
│
├── odes/ # ODE functions for simulation (ode45-ready)
│ ├── P/ # Pendulum models (linear and spherical)
│ ├── MSD/ # Mass-spring-damper models
│ └── Parameters.m # Parameter definitions and variations
│
├── Models Validation/ # Analysis on model behavior 
│
└── Parameters Estimation/ # Model parameters estimation based of FT readings
    └── bagfiles/ # Contain Rosbags of the FT sensor readings
```

---

## 📁 `mlx/`

This directory uses MATLAB Live Scripts to symbolically derive equations of motion (EOM) for various sloshing models under different excitation conditions (3D and 4D).

### Subfolders:

- **YX/**: Spherical pendulum model using YX Euler angle parametrization.
- **ZY/**: Spherical pendulum model using ZY parametrization.
- **Linear Pendulum/**: Classical linear pendulum.
- **MSD/**: Mass-Spring-Damper (MSD) system.

Each subfolder typically contains:
- Derivation scripts for specific excitation types (e.g., 3D or 4D base motion)
- Symbolic simplifications and linearization (where applicable)
- Automated EoMs ode implementation (where applicable)

---

## 📁 `odes/`

This directory includes MATLAB `.m` function files implementing the EOM derived in `mlx/`, designed for numerical integration using solvers like `ode45`.

### Subfolders:

- **P/**:
  - Contains dynamics functions for pendulum-based sloshing models.
  - Supports YX, ZY, and linear pendulum parametrizations.
  - Compatible with 3D and 4D excitation.
  - The "main" files used (also in the relative Scientific Paper) are `odeSchoenP.m` and `odeTiltP.m`
  
- **MSD/**:
  - Implements ODEs for mass-spring-damper models of sloshing.
  - The "main" files used (also in the relative Scientific Paper) are `odeSchoenMSD.m` and `odeTiltMSD.m`
  - A subfolder `MLXgen` contains the EoMs generated via the MLX files, their derivation is not to be assumed 100% correct

- Contains `Parameters.m` and related variations, defining parameter sets for all models.

Note: each model subfolder contains a `RK` folder in which the odes are expressed with Runge-Kutta integration, this avoids using `ode45` and is generally faster.

---

## 📁 `Models Validation/`

This directory includes MATLAB `.m` files used to validate the developed models, in terms of quantitative and qualitative analysis wrt experimental data. Multiple files are present, aiming to understand the validity of the discrete models, the parameters useful in the scenarios considered, the number of sloshing masses to be considered and the pendulum sloshing height formulation to be employed.

---

## 📁 `Parameters Estimation/`

This directory includes MATLAB `.m` files used to estimate the discrete model parameters based on the liquid experimental behavior. The rosbag files including forct/torque sensor readings and joint state readings are imported and analyzed in order to estimate damping factor and natural frequency of the first mode of oscillation of the liquid.
- `JointsBag.m` imports and visualizes joint states (note that the velocity part of the topic is not correct)
- `ParametersEstimation.m` imports the force/torque readings, and estimates damping ratio and natural frequency.
- `Rotz_Trascinamento.m` estimates the dragging factor of the liquid in a cylindircal container following a rotation about its vertical axis.

---

## Usage Notes

- Run and verify the `.mlx` files before using their results in numerical code, i.e. check that the sign of the rotation angles correspond to the wanted ones.
- The `odes/` functions expect properly dimensioned initial states and parameter structures. See code examples for expected input formats.
- Import errors may be present as this codebase has been assembled from different sources. All the relevant files however are given, so be sure to update the import path in order to correctly run the given scripts.

---

## Dependencies

- MATLAB R2021a or later (for full symbolic support and Live Scripts)
- **ROS Toolbox** (for rosbag import)
- Symbolic Math Toolbox (for `mlx`)
- No external toolboxes required for `odes/`