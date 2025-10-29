# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository implements trajectory planning for multi-container non-prehensile transport using a robotic manipulator. The code solves three distinct scenarios:

1. **Anti-sloshing optimization** (`NonPrehensile_Assigned_Path_Optimization.m`): Time-optimal, path-parameterized (s), liquid-filled containers
2. **Rigid body optimization** (`NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m`): Time-optimal, path-parameterized (s), solid containers with tilt
3. **Direct Torque OCP D-T** (`Prehensile_New_OCP.m`): Direct torque control, time-based, sloshing suppression with joint trajectory optimization

All formulate problems as non-linear optimal control problems (OCP) using CasADi's Opti stack, discretized via multiple shooting and solved with IPOPT.

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

Run any of the main scripts:
```matlab
NonPrehensile_Assigned_Path_Optimization.m                    % Anti-sloshing (path-param)
NonPrehensile_Assigned_Path_Multi_Rigid_Body_Optimization.m   % Rigid body with tilt (path-param)
Prehensile_New_OCP.m                          % Direct torque OCP D-T (time-based)
```

**Path-Parameterized Scripts** follow an **interactive GUI workflow** with 5 sequential dialog windows for configuration:
1. Save options (figures, CSV trajectories, MAT files) and animation preferences
2. Container and liquid properties (geometry, mass, sloshing limits)
3. Container placement on tray (number, positions, tray dimensions, friction)
4. Tray-to-end-effector transformation
5. Prescribed geometric path definition and orientation

After configuration, the optimization runs automatically and generates plots and data files.

**Direct Torque OCP D-T Script** (`Prehensile_New_OCP.m`) supports two execution modes:
- **Quick-test mode**: Uses default parameters (4 containers, 5s horizon, 100Hz discretization) without GUI prompts for rapid prototyping
- **Interactive GUI mode**: Same 4 configuration dialogs as path-parameterized scripts (1-4 above), plus:
  5. **Reference pose trajectory** (position + yaw angle vs time) instead of geometric path definition

The reference pose can be loaded from file or defined via piecewise-constant, linear interpolation, or spline representation. The optimization then finds joint torques to track this reference while suppressing sloshing.

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

### Main Script Structure Pattern (Path-Parameterized Scripts)

Path-parameterized optimization scripts follow this workflow:

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

### OCP D-T Script Structure (`Prehensile_New_OCP.m`)

The Direct Torque OCP D-T script follows a different architecture optimized for trajectory tracking with sloshing suppression:

**Quick-test mode** (no user interaction):
1. Load libraries and set platform-specific paths
2. Use default configuration (4 containers, 5.0s horizon, 0.1s timestep)
3. Skip all GUI dialogs
4. Proceed directly to step 5 below

**Interactive GUI mode**:
1. Load libraries and set platform-specific paths
2. Configuration via 4 GUI dialogs (save options, container properties, placement, tool offset)
3. Display available reference trajectories and load/define reference pose trajectory
4. Robot model instantiation and tool offset setup
5. Proceed to dynamics and optimization setup below

**Dynamics and Optimization Setup** (common for both modes):
5. Create CasADi symbolic dynamics via `createDynamicsCasadiFcn.m`:
   - Full robot inertia, Coriolis, gravity matrices M(q), C(q,q̇), G(q)
   - End-effector kinematics and accelerations
   - Container sloshing dynamics (MSD model) for each container
   - RK4 discrete-time integration `x_{k+1} = RK4(x_k, u_k, dt)`
6. Setup Opti problem with scaled decision variables (q, q_dot, u):
   - Decision variables: `X = [q(0), q_dot(0), slosh(0), ..., u(0), u(1), ..., u(N-1)]`
   - Discretized state propagation constraints: `X_{k+1} = RK4(X_k, u_k, dt)`
7. Define multi-objective cost function:
   - Stage cost: Position tracking + Yaw tracking + Sloshing suppression + Control effort
   - Terminal cost: Position tracking + Yaw tracking + Sloshing suppression + Final velocity
8. Add constraints:
   - Joint position limits: `q_min ≤ q_k ≤ q_max`
   - Joint torque limits: `τ_min ≤ u_k ≤ τ_max`
   - Sloshing height limits: `||slosh_i,k|| ≤ η_max` (per container)
9. Solve with IPOPT
10. Post-processing: Verify constraints, plot results (joint trajectories, end-effector motion, sloshing heights, cost evolution)

**Key Differences from Path-Parameterized Approach**:
- **Direct torque control**: Optimizes joint torques directly (not path jerk)
- **Time-based**: State evolution explicit in time t, not path progress s
- **Full dynamics**: Includes robot inertia, Coriolis, gravity terms
- **Pose tracking**: References are end-effector position + yaw (not geometric path)
- **Simplified constraints**: No waiter problem (non-prehensile contact constraints)

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

## OCP D-T: Direct Torque Control Formulation

### Overview

The **Direct Torque OCP D-T** (`Prehensile_New_OCP.m`) is a new formulation that differs fundamentally from the path-parameterized approaches:

| Aspect | Path-Param (s) | Direct Torque (D-T) |
|--------|---|---|
| **Control** | Path jerk (s⃛) | Joint torques (τ) |
| **Parameterization** | Path parameter s ∈ [0,1] | Time t ∈ [0, T] |
| **Objective** | Time-optimal | Trajectory tracking + sloshing suppression |
| **State** | [s, ṡ, s̈, sloshing] | [q, q̇, sloshing] |
| **Constraints** | Waiter (contact) | Joint limits, torque limits, sloshing limits |

### Mathematical Formulation

**Problem statement** (from OCP.pdf):
```
minimize: sum_t L(x_t, u_t) + L_f(x_T)
subject to: x_{t+1} = f(x_t, u_t)
            constraints: joint limits, torque limits, sloshing limits
```

**State vector** (R^(12 + 4·num_containers)):
```
x_t = [q_t^T, q̇_t^T, slosh_1,t, slosh_2,t, ..., slosh_M,t]^T
```
where:
- `q, q̇` ∈ R^6: Joint positions and velocities
- `slosh_i,t ∈ R^4` for i = 1,...,M: Sloshing state for container i
  - `slosh_i = [x_i, y_i, ẋ_i, ẏ_i]^T` (horizontal position and velocity of sloshing mass)
  - M = `num_containers` (default: 4 in quick_test mode, configurable via GUI)

**Control vector** (R^6):
```
u_t = τ_t  (joint torques)
```

**Cost function**:

Stage cost at each time step t:
```
L(x_t, u_t) = ½||p_EE(q_t) - p_ref,t||²_Qp           % End-effector position tracking
            + ½||yaw(q_t) - yaw_ref,t||²_Qrot        % Yaw angle tracking
            + ½ Σᵢ ||η_i(x_t)||²_Qη                  % Sloshing suppression (all containers)
            + ½||u_t||²_R                             % Control effort regularization
```

Terminal cost at final time T:
```
L_f(x_T) = ½||p_EE(q_T) - p_ref,T||²_Pp             % Final position
         + ½||yaw(q_T) - yaw_ref,T||²_Prot           % Final yaw
         + ½ Σᵢ ||η_i(x_T)||²_Pη                     % Final sloshing (all containers)
         + ½||q̇_T||²_Pq̇                             % Final joint velocities
```

where `η_i(x) = (ξ²₁₁ h m₁ / (m_f R)) * sqrt(x² + y²)` is the sloshing height for container i.

### Continuous Dynamics

The continuous-time dynamics ẋ = f_cont(x, u) are computed in 5 steps:

1. **Robot Dynamics**: Compute joint accelerations from inverse dynamics
   ```
   q̈ = M(q)⁻¹ [τ - C(q, q̇)q̇ - G(q)]
   ```

2. **End-Effector Kinematics**: Compute EE velocities and accelerations
   ```
   [ṙ^T, ω^T]^T = J(q) q̇
   [r̈^T, ω̇^T]^T = J̇(q, q̇)q̇ + J(q)q̈
   ```

3. **Container Accelerations**: Compute accelerations of each container center
   ```
   r̈_i = r̈ + ω̇ × d_i + ω × (ω × d_i)
   ```

4. **Sloshing Dynamics**: Solve coupled nonlinear ODE for sloshing accelerations
   ```
   [1 + P_n²x² , P_n²xy  ] [ẍ]   [a_i]
   [P_n²xy    , 1 + P_n²y²] [ÿ] = [b_i]
   ```

5. **State Derivative**: Construct ẋ = [q̇^T, q̈^T, ẋ_IC, ẏ_IC, ẍ_IC, ÿ_IC, ẋ_EC, ẏ_EC, ẍ_EC, ÿ_EC]^T

The dynamics are discretized using **RK4 integration** with N time steps:
```
x_{t+1} = RK4(ẋ, x_t, u_t, Δt)
```

### Key Differences from Path-Parameterized Approach

1. **Direct Robot Control**: Optimizes joint torques directly instead of path parameter jerk
2. **Full Robot Dynamics**: Includes inertia, Coriolis, and gravity terms (M, C, G matrices)
3. **Time-Variable Formulation**: State explicitly depends on time, not path progress
4. **Sloshing-Focused Objective**: Minimizes sloshing and control effort, not trajectory time
5. **Simplified Constraints**: No waiter problem constraints; focus on joint/torque limits and sloshing

### Implementation Notes

**Files for OCP D-T** (Updated October 29, 2025):
- `Prehensile_New_OCP.m` - Main optimization script with pose tracking and sloshing suppression ✅
- `include/createDynamicsCasadiFcn.m` - CasADi symbolic dynamics & RK4 integration ✅
- `include/setup/getReferencePoseTrajectory.m` - GUI for reference pose trajectory (position + yaw) setup ✅

**Optimization parameters** (in main script):
- `N` - Number of control intervals, computed as `N = round(T_fixed / dt)` (default: 50 with T=5.0s, dt=0.1s)

**Stage cost weights**:
- `Qp` - Position tracking weight (100 by default, 3×3 matrix)
- `Qrot` - Yaw angle tracking weight (100 by default, scalar)
- `Qeta` - Sloshing suppression weight per container (100 by default)
- `R` - Joint torque effort weight (0.01 by default, 6×6 matrix)

**Terminal cost weights**:
- `Pp` - Final position tracking weight (1000 by default, 3×3 matrix)
- `Prot` - Final yaw tracking weight (1000 by default, scalar)
- `Peta` - Final sloshing weight per container (1000 by default)
- `Pqdot` - Final joint velocity weight (100 by default, 6×6 matrix)

**✅ Dynamics Implementation - COMPLETED**:

The OCP D-T implementation now includes **full symbolic dynamics integration**:

1. **Robot Dynamics Matrices**: M(q), C(q, q̇), G(q) are computed using:
   - Configuration-dependent inertia matrix based on SmartSix DH parameters
   - Velocity-dependent Coriolis and centrifugal terms
   - Gravity vector accounting for link masses and positions
   - Supports both analytical approximation and rigidBodyTree (when available)

2. **CasADi Symbolic Formulation**: All dynamics are fully symbolic via `createDynamicsCasadiFcn.m`:
   - Returns `dynamics_fcn`: continuous dynamics x_dot = f(x, u)
   - Returns `rk4_fcn`: discrete RK4 integration x_{k+1} = RK4(x_k, u_k, dt)
   - Automatic differentiation enabled for IPOPT gradients
   - Used in optimization: `opti.subject_to(X(:, k+1) == rk4_fcn(x_k, u_k, dt))`

3. **Validation Utilities**: `computeRobotDynamics.m` provides:
   - Ability to use MATLAB rigidBodyTree for accurate dynamics (when available)
   - Improved analytical approximation as fallback
   - Consistent interface for both modes

**Remaining Enhancements** (optional):
- Use analytical Jacobian derivative (currently numerical finite differences)
- Add waiter problem constraints if contact preservation is needed
- Implement EE pose tracking constraints for trajectory following
- Fine-tune optimization weights and scaling
