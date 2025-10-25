# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository implements time-optimal trajectory planning for multi-container non-prehensile transport using a robotic manipulator. The code solves two distinct scenarios:

1. **Anti-sloshing optimization** (`NonPrehensile_Assigned_Path_Optimization.m`): Liquid-filled containers with mass-spring-damper sloshing dynamics
2. **Rigid body optimization** (`NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m`): Solid containers with additional tilt degrees of freedom

Both formulate the problem as a non-linear optimal control problem (OCP) using CasADi's Opti stack, discretized via multiple shooting and solved with IPOPT.

## Running the Code

### Environment Setup

Open MATLAB 2023b (or compatible) and navigate to the project root directory. The code automatically detects the platform and adds the appropriate CasADi path:

```matlab
win = 0;  % Set to 1 for Windows, 0 for Linux
```

- **Linux**: Uses `casadi-3.6.5-linux64-matlab2018b/`
- **Windows**: Uses `casadi-3.6.5-windows64-matlab2018b/`

The `include/`, `Data/`, and appropriate CasADi directories are added to the MATLAB path automatically.

### Execution

Run either main script:
```matlab
NonPrehensile_Assigned_Path_Optimization.m              % Anti-sloshing
NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m  % Rigid body with tilt
```

Both scripts follow an **interactive GUI workflow** with 5 sequential dialog windows for configuration:
1. Save options (figures, CSV trajectories, MAT files) and animation preferences
2. Container and liquid properties (geometry, mass, sloshing limits)
3. Container placement on tray (number, positions, tray dimensions, friction)
4. Tray-to-end-effector transformation
5. Prescribed geometric path definition and orientation

After configuration, the optimization runs automatically and generates plots and data files.

## Core Architecture

### Trajectory Parameterization

The geometric path is **prescribed** and parameterized by scalar coordinate **s ∈ [0,1]**. The optimization determines the time evolution of s to minimize trajectory time while respecting constraints.

**Control variables:**
- Anti-sloshing script: `u = s_dddot` (jerk of path parameter)
- Rigid body script: `u = [s_dddot, theta_x_dddot, theta_y_dddot]` (path jerk + pitch/roll jerks)

**State variables:**
- Anti-sloshing: `X = [s, s_dot, s_ddot, xy_slosh_positions, xy_slosh_velocities]` (per container)
- Rigid body: `X = [s, s_dot, s_ddot, theta_x, theta_y, theta_x_dot, theta_y_dot, theta_x_ddot, theta_y_ddot]`

The yaw angle θ_z is always prescribed as a linear function of s, but pitch and roll can be optimized in the rigid body case.

### CasADi Optimization Structure

Both scripts use the same optimization pattern:

1. **Define symbolic variables** for path parameters and create CasADi Functions relating s (and angles) to robot quantities via kinematics
2. **Setup Opti problem** with scaled decision variables (improves convergence)
3. **Define cost function**: `minimize(T + regularization * control_effort)`
4. **Discretize dynamics** using RK4 integration with multiple shooting (N=150 intervals)
5. **Add constraints**: joint limits, non-prehensile conditions, sloshing limits (if applicable)
6. **Solve with IPOPT** using callbacks to visualize s evolution during optimization

### Key CasADi Functions

The codebase creates numerous CasADi Function objects that encapsulate the kinematic and dynamic relationships. These are defined in `include/formulate*.m` scripts and then "mapped" across all time steps using CasADi's `.map()` for vectorized evaluation:

```matlab
% Example pattern from the code:
get_q_dot = casadi.Function('get_q_dot', {s, s_dot}, {jacobian(q,s)*s_dot});
q_dot_map = get_q_dot.map(N+1);  % Vectorize across N+1 time steps
q_dot = q_dot_map(s_o, s_dot_o);  % Evaluate for entire trajectory
opti.subject_to(q_dot_min <= q_dot <= q_dot_max);
```

This pattern appears throughout: create symbolic function → map it → apply constraints.

### Liquid Sloshing Dynamics (Anti-sloshing script only)

The liquid is modeled as a 4D mass-spring-damper (MSD) system per container with coupled nonlinear equations of motion. The system tracks the position `[x_s, y_s]` and velocity `[x_s_dot, y_s_dot]` of the equivalent sloshing mass.

**Key parameters** (computed in `Parameters.m` function):
- `wn`: natural frequency
- `zita`: damping ratio
- `mn`: equivalent mass
- `as`: nonlinearity coefficient
- `csi11`: mode shape parameter

The sloshing height constraint is: `sqrt(x_s^2 + y_s^2) <= eta_lim * scaling_factor`

After optimization, actual sloshing heights are computed by numerical integration using `ode45` with the nonlinear MSD model (`odefunctionNL_MSD.m`).

### Waiter Problem Constraints

Four constraint types prevent objects from moving relative to the tray, formulated in the tray frame:

1. **Non-lift**: `F_z <= 0` (normal force must be compressive)
2. **Non-slip**: `F_x^2 + F_y^2 <= μ^2 * F_z^2` (friction cone)
3. **Non-tip**: `F_x^2 + F_y^2 <= (r/h_G)^2 * F_z^2` (tipping stability)
4. **Non-twist**: `|M_z| <= (2/3) * μ * |F_z| * r` (torsional stability)

Forces and moments account for:
- Translational acceleration of container center of mass
- Rotational dynamics (Euler's equation)
- Gravity projection into tray frame

These are evaluated at every time step across all containers.

## Important Development Considerations

### Constraint Function Derivatives

**Critical**: IPOPT uses gradient information, so constraint functions must have well-defined derivatives everywhere, especially at the initial guess.

The derivative of `sqrt(x)` approaches infinity as x→0, causing numerical instability. If constraints involve square roots, ensure the initial guess keeps arguments bounded away from zero or reformulate as squared constraints where possible.

**Debug check**: `opti.debug.value(jacobian(constraint, X), opti.initial())`

### Variable Scaling

Decision variables are scaled by representative magnitudes to normalize the problem:

```matlab
% Example from anti-sloshing script:
X = repmat([1; 1; 2; 0.01*ones(x_dim-3,1)], 1, N+1) .* opti.variable(x_dim, N+1);
```

This improves convergence by bringing all variables to similar orders of magnitude.

### File Organization

- **`include/Comau_Kinematics/`**: SmartSix robot class with forward/inverse kinematics, Jacobians
- **`include/formulate*.m`**: Scripts that define CasADi symbolic functions for trajectories, kinematics, dynamics
- **`include/setup/`**: GUI dialog functions for interactive parameter input
- **`include/MSD_odes/`**: ODE functions for post-optimization sloshing simulation
- **`include/Tilt/`**: Alternative formulations for rigid body optimization with tilt
- **`include/useful_functions/`**: Utility functions (rotation matrices, skew-symmetric, etc.)
- **`include/debug/`**: Plotting and diagnostic scripts executed via `run()` statements

### Main Script Structure Pattern

Both optimization scripts follow this workflow:

1. Load libraries and set platform-specific paths
2. Interactive GUI configuration (5 dialog windows)
3. Robot model instantiation (`SmartSix()` class)
4. Tray-to-end-effector transformation setup (`T67`)
5. Trajectory formulation (`run("formulateTrajectory.m")`)
6. Angular trajectory formulation (`run("formulateAngularTrajectory.m")`)
7. Constraint formulation (waiter problem, sloshing if applicable)
8. Optimization problem setup (Opti stack)
9. Solve with IPOPT
10. Post-processing: sloshing simulation, inverse kinematics verification, plotting
11. Optional CSV export at 500 Hz for robot execution

### CasADi Debugging

The scripts use `opti.callback(@(i) plot(opti.debug.value(s_o)))` to visualize s evolution during optimization.

If optimization fails, use:
```matlab
sol = opti.debug();
sol.show_infeasibilities(1e-4);  % Show violated constraints
opti.debug.x_describe(index);    % Identify problematic variable
opti.debug.g_describe(index);    % Identify problematic constraint
```

See `CasaDi_Notes.md` for comprehensive debugging strategies.

### Robot Model

The `SmartSix()` class (in `include/Comau_Kinematics/`) wraps the Comau Smart Six robot and provides:
- Forward kinematics: `fk(q)`
- Inverse kinematics: `ik(T, elbow_config)`
- Analytical Jacobian: `jacob_a(q)`
- Joint limits: `q_min`, `q_max`, `qp_min`, `qp_max`, `qpp_min`, `qpp_max`

For visualization, the script uses Robotics System Toolbox's `rigidBodyTree` imported from `comau_smartsix5.urdf.xacro`.

### Output Files

**Figures** (saved to `Plots/` or `Plots/Tilt/`):
- Trajectory preview and execution animations (if enabled)
- Sloshing heights vs time (anti-sloshing only)
- Waiter constraint margins (4-panel subplot)
- Joint angles, velocities, accelerations
- End-effector and tray kinematic quantities
- Solver diagnostics (objective evolution, feasibility)

**Data files**:
- CSV: Joint angle trajectories at 500 Hz (`Data/CSV/`)
- MAT: Sloshing height time series per container (`Data/etas/`)

File naming: `TO_<path_type>_<eta_lim>_<num_containers>containers_<angle>_deg.*`

## Testing and Validation

After optimization, the scripts automatically verify:
1. Joint position limits: `q_min < q < q_max`
2. Joint velocity limits: `q_dot_min < q_dot < q_dot_max`
3. Self-collision check at all time steps using `checkCollision()`
4. Inverse kinematics consistency (compare optimized q with IK solution)

Errors are raised if any validation fails.

## Path Definition

Geometric paths are defined in `include/getPath.m` with preset options (straight line, arc, S-curve) or custom user input. Paths are internally represented as splines with control points `[cpx, cpy, cpz]`.

The trajectory formulation scripts create CasADi Functions that map `s` and its derivatives to:
- Position: `get_p_07_0(s)`
- Velocity: `get_v_07_0(s, s_dot)`
- Acceleration: `get_a_07_0(s, s_dot, s_ddot)`

These are then combined with rotation matrices to get full 6-DOF motion.
