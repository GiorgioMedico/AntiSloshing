# NonPrehensile_Assigned_Path_Optimization.m - Complete Section-by-Section Breakdown

## Overview

This MATLAB script solves a **time-optimal trajectory planning problem** for a Comau Smart Six robotic manipulator carrying multiple liquid-filled containers on a tray. The robot must move along a prescribed geometric path while:
- Minimizing total trajectory time
- Preventing liquid sloshing beyond a threshold
- Ensuring containers don't slip, lift, tip, or twist relative to the tray

The script uses **CasADi's Opti stack** to formulate the problem as a nonlinear optimal control problem (OCP), discretized via multiple shooting, and solved with the IPOPT solver.

---

## Section-by-Section Breakdown

### Section 1: Library Loading & Platform Setup (Lines 10-26)

**What it does:**
- Clears MATLAB workspace, closes figures, and clears persistent classes
- Adds necessary directories to MATLAB path for data, helper functions, and CasADi library
- Auto-detects platform (Windows or Linux) and adds appropriate CasADi binaries
- Imports CasADi namespace for symbolic computation

**Why it's needed:**
CasADi is the core optimization framework. The code must be compiled for each platform (Windows/Linux).

**Key variables:**
- `win = 0`: Flag indicating Linux environment (set to 1 for Windows)
- `addpath()`: Adds directories to search path for helper functions

**Platform paths:**
```
Windows: casadi-3.6.5-windows64-matlab2018b/
Linux:   casadi-3.6.5-linux64-matlab2018b/
```

---

### Section 2: Simulation Flags & Configuration (Lines 28-34)

**What it does:**
- Calls `getSimulationParameters()` to gather user preferences for output
- Sets flags for saving figures, CSV trajectories, MAT files, and animations
- Initializes `is_solved` flag to track optimization success
- Sets output folder and robot URDF file

**Key variables:**
- `save_fig`: Boolean to save plots as PDF
- `save_csv`: Boolean to save joint trajectories as CSV
- `save_mat`: Boolean to save sloshing data as MAT
- `csv_joint_offset`: Initial joint configuration offset
- `animations`: Boolean to generate preview/execution animations
- `fig_name`: Prefix for all output file names

---

### Section 3: Container & Liquid Physics Setup (Lines 37-59)

**What it does:**
- **Interactive GUI dialog** (`getAndVisualizeContainerParameters`) prompts user to select:
  - Cylinder radius and height
  - Liquid fill level
  - Sloshing limit (η_lim)
  - Total container mass
  - Liquid density and viscosity
- Computes liquid physical parameters using `Parameters()` function:
  - Natural frequency (wn)
  - Damping ratio (zita)
  - Equivalent sloshing mass (mn)
  - Nonlinearity coefficients (ks, cs, as)
  - Mode shape parameter (csi11)
- Calculates container inertia tensor (Ixx, Iyy, Izz) about center of mass

**Why it's needed:**
The sloshing dynamics are modeled as a 4D nonlinear mass-spring-damper system. These parameters define how the liquid responds to acceleration.

**Key variables:**
- `cyl_radius, fill_level, eta_lim`: Container geometry and sloshing threshold
- `g = 9.81`: Gravitational acceleration
- `m_fluid`: Mass of liquid
- `Vol`: Container volume
- `wn`: Natural frequency of sloshing mode
- `zita`: Damping ratio
- `mn`: Equivalent sloshing mass
- `as`: Nonlinearity coefficient (affects sloshing amplitude)
- `IG_G`: 3×3 inertia matrix about container center of mass

**Physical model:**
The container is modeled as a rigid cylinder with:
- Uniform density
- Liquid oscillations treated as concentrated mass at equivalent height

---

### Section 4: Tray & Container Placement (Lines 60-80)

**What it does:**
- **Interactive GUI dialog** prompts user for:
  - Number of containers (num_containers)
  - Placement mode: manual (specify positions) or automatic (grid/circle/custom)
  - Tray dimensions (length, width, thickness)
  - Friction coefficient between container and tray
- For **manual mode**: `get2DVectorInput()` requests 2D positions for each container
- Calls `getContainerPlacement()` to compute:
  - Position vectors of each container relative to tray center (p_7G_all)
  - Positions in base frame (p_7b_all)
  - Diagonal distances (p_diag)
- Sets tray-to-container rotation matrix (R7G = eye(3) for zero rotation)

**Why it's needed:**
The positions and number of containers determine:
- How forces/moments distribute across the tray
- Constraint equations for slip, tip, and twist

**Key variables:**
- `num_containers`: Number of containers
- `pMode`: Placement mode ("manual" or automatic options)
- `p_7G_all`: 3×num_containers matrix of container positions in tray frame
- `p_7b_all`: Container positions in base/world frame
- `tray`: Struct containing tray properties (length, width, thickness, friction)
- `R7G`: Rotation matrix from container frame to tray frame

---

### Section 5: Robot Model Initialization (Lines 82-108)

**What it does:**
- Creates Comau Smart Six robot instance via `SmartSix()` class
- **Interactive GUI dialog** (`getToolOffsetParameters`) defines tray-to-end-effector frame transformation:
  - Translation offset from EE to tray center (toolOffset)
  - Rotation offset about z-axis (toolRotationOffset)
- Constructs transformation matrix T67 (tray frame to EE frame)
- Computes inertia matrix in tray frame (IG_7) and EE frame (IG_6)
- Visualizes complete setup (robot + tray + containers) via `visualizeSetup()`

**Why it's needed:**
The robot must know how to position/orient the tray relative to its end-effector. This transformation is critical for kinematics.

**Key variables:**
- `robot`: SmartSix robot object with kinematics, Jacobians, and joint limits
- `toolOffset`: 3×1 vector [x, y, z] offset from EE to tray
- `toolRotationOffset`: Scalar yaw rotation about z-axis
- `T67`: 4×4 homogeneous transformation from frame 6 (EE) to frame 7 (tray)
- `IG_7, IG_6`: Inertia tensors in tray and EE frames (used for moment calculations)
- `robot_visu`: Visualization object (rigidBodyTree)

---

### Section 6: Robot Limits Definition (Lines 110-121)

**What it does:**
- Defines path parameter limits:
  - `s ∈ [0, 1]`: Path starts at s=0, ends at s=1
  - `s_dot ≥ 0`: Path parameter velocity is non-negative (forward motion only)
- Sets control input limits on jerk:
  - `u_min = -s_j_lim, u_max = +s_j_lim` (default ±1000)
- Applies robot joint limits with safety factor:
  - Position: `q_min, q_max` from robot spec
  - Velocity: `q_dot_min, q_dot_max` (with safety factor ~0.8)
  - Acceleration: `q_ddot_min, q_ddot_max` (with safety factor ~0.8)

**Why it's needed:**
IPOPT uses these bounds to constrain the optimization variables to physically and mechanically feasible ranges.

**Key variables:**
- `s_min, s_max`: Path parameter bounds
- `s_j_lim`: Jerk magnitude limit (control input)
- `u_min, u_max`: Control input bounds
- `q_min, q_max`: Joint position limits (radians)
- `q_dot_min, q_dot_max`: Joint velocity limits (rad/s)
- `q_ddot_min, q_ddot_max`: Joint acceleration limits (rad/s²)
- `safety`: Factor < 1 to provide margin below robot specifications

---

### Section 7: Geometric Path Formulation (Lines 123-136)

**What it does:**
- **Interactive GUI dialog** (`getGeometricPathParameters`) collects:
  - Path type (straight line, circular arc, S-curve, custom)
  - Orientation evolution (yaw angle θ_z(s))
  - Control points for spline interpolation (cpx, cpy, cpz)
  - Number of control points
- Executes `formulateTrajectory.m` to create CasADi Functions:
  - `get_p_07_0(s)`: Position of tray w.r.t. base as function of s
  - `get_v_07_0(s, s_dot)`: Velocity as function of s and ṡ
  - `get_a_07_0(s, s_dot, s_ddot)`: Acceleration as function of s, ṡ, s̈

**Why it's needed:**
The geometric path is **fixed/prescribed** (not optimized). The script only optimizes the time evolution s(t). This reduces problem dimensionality significantly.

**Key variables:**
- `path_type`: String name of curve (e.g., "line", "arc")
- `cpx, cpy, cpz`: Control points for spline
- `n_ctrl_pts`: Number of control points
- `get_p_07_0, get_v_07_0, get_a_07_0`: CasADi Function handles

**Naming convention:**
Figure names updated to include path type and orientation: `TO_<path_type>_<eta_lim>_<num_containers>containers_<angle>_deg`

---

### Section 8: Angular Trajectory Formulation (Lines 138-148)

**What it does:**
- Executes `formulateAngularTrajectory.m` to create CasADi Functions for yaw rotation:
  - `get_thz_0(s)`: Yaw angle θ_z as function of s
  - `get_thz_dot_0(s, s_dot)`: Angular velocity θ̇_z as function of s, ṡ
  - `get_thz_ddot_0(s, s_dot, s_ddot)`: Angular acceleration θ̈_z as function of s, ṡ, s̈
- Executes `formulateAngularQuantities.m` to compute:
  - Rotation matrix R07(s) from base to tray (yaw only)
  - Angular velocity vector w = [0, 0, θ̇_z]
  - Angular acceleration vector α = [0, 0, θ̈_z]

**Why it's needed:**
Rotational kinematics are needed for:
- Force/moment transformations into tray frame (for waiter constraints)
- Container acceleration calculations
- Sloshing dynamics (depends on container acceleration in tray frame)

**Key variables:**
- `thz`: Yaw angle (scalar function of s)
- `thz_dot, thz_ddot`: Yaw velocity and acceleration
- `get_thz_0, get_thz_dot_0, get_thz_ddot_0`: CasADi Functions
- `R07`: 3×3 rotation matrix from base to tray

---

### Section 9: End-Effector Kinematics (Lines 149-150)

**What it does:**
- Executes `formulateEndEffectorKinematics.m` to create CasADi Functions for EE motion:
  - Maps tray motion through transformation T67 to compute EE position, velocity, acceleration
  - Creates functions like `get_p_06_0(s)`, `get_v_06_0(s, s_dot)`, `get_a_06_0(s, s_dot, s_ddot)`

**Why it's needed:**
Inverse kinematics require full 6-DOF EE pose. This section converts tray trajectory to EE requirements.

---

### Section 10: Differential Kinematics & Joint Mapping (Lines 152-154)

**What it does:**
- Executes `formulateDiffKinDep.m` to create CasADi Functions mapping path parameters to joint variables:
  - `get_q(s)`: Joint angles as function of path parameter
  - `get_q_dot(s, s_dot)`: Joint velocities as function of s, ṡ
  - `get_q_ddot(s, s_dot, s_ddot)`: Joint accelerations as function of s, ṡ, s̈
- Uses robot's analytical Jacobian: `J = ∂q/∂s`
- Thus: `q̇ = J * ṡ` and `q̈ = J * s̈ + (∂J/∂s * ṡ) * ṡ`

**Why it's needed:**
Joint limits are applied to constrain q, q̇, q̈, which are all derived from the path parameter s and its derivatives.

**Key variables:**
- `get_q, get_q_dot, get_q_ddot`: CasADi Functions

---

### Section 11: Trajectory Preview & Sanity Check (Lines 157-160)

**What it does:**
- **Optional step** (if `animations = true`):
  - Runs `plotTrajPreviewAnimation.m`: Animates robot following the prescribed path
  - Runs `plotPreview.m`: Static plots of path, velocities, accelerations
- Allows user to visually verify path makes sense before optimization

---

### Section 12: Initial & Final Conditions Setup (Lines 162-185)

**What it does:**
- Defines optimization problem dimensions:
  - `x_dim = 3 + 4*num_containers`: State vector size
    - 3 for [s, ṡ, s̈]
    - 4 per container for [x_slosh, y_slosh, ẋ_slosh, ẏ_slosh]
  - `u_dim = 1`: Control input is jerk u = s⃛
- Creates initial and final state vectors:
  - `x_start_OCP = [0, 0, 0, 0, ...]`: Starts at s=0, rest at zero
  - `x_end_OCP = [1, 0, 0, 0, ...]`: Ends at s=1, all else zero (static sloshing)
- Computes IK to find joint configurations at start/end points
- Plots initial/final robot configurations and EE poses

**Why it's needed:**
These boundary conditions constrain the optimization to start at s=0 (path beginning) and end at s=1 (path end) with sloshing settled.

**Key variables:**
- `x_dim`: Total state dimension
- `u_dim`: Control dimension (always 1 here)
- `x_start_OCP, x_end_OCP`: Initial and final state vectors
- `q_0, q_fin`: Joint angles at start and end
- `T_start, T_end`: EE transformation matrices at boundaries

---

### Section 13: Waiter Constraint Formulation (Lines 191-244)

**What it does:**
This section formulates the **four constraints preventing containers from moving relative to the tray**. For each container i:

**Physics setup:**
- Computes container acceleration in world frame:
  ```
  a_container = a_tray + α × r + ω × (ω × r)
  ```
  where r = position of container relative to tray center
- Transforms gravity and inertial forces into tray frame
- Applies Euler's equation for rotational dynamics

**Four constraint functions created:**

1. **Non-lift constraint** (`nonLift`):
   - Normal force must be compressive: F_z ≤ 0
   - Prevents container from lifting off tray

2. **Non-slip constraint** (`nonSlip`):
   - Friction cone: F_x² + F_y² ≤ μ² * F_z²
   - Prevents sliding in horizontal plane

3. **Non-tip constraint** (`nonTip`):
   - Stability against tipping: F_x² + F_y² ≤ (R / h_G)² * F_z²
   - Where R = cylinder radius, h_G = height of CoM above tray
   - Prevents overturning

4. **Non-twist constraint** (`nonTwist`):
   - Torsional stability: |M_z| ≤ (2/3) * μ * |F_z| * R
   - Prevents spinning about vertical axis

**CasADi functions created:**
- `get_nonLift(s, ṡ, s̈)`: Returns num_containers × 1 vector
- `get_nonSlip(s, ṡ, s̈)`: Returns num_containers × 1 vector
- `get_nonTip(s, ṡ, s̈)`: Returns num_containers × 1 vector
- `get_nonTwist(s, ṡ, s̈)`: Returns num_containers × 1 vector

**Key variables:**
- `nonLift, nonSlip, nonTip, nonTwist`: Constraint vectors (one per container)
- `get_nonLift, get_nonSlip, get_nonTip, get_nonTwist`: CasADi Function handles
- `xdd, fg_7, f_7`: Container acceleration, gravity force, total force in tray frame

---

### Section 14: Optimization Problem Setup (Lines 246-267)

**What it does:**
- Creates `opti = casadi.Opti()` optimization object
- **Defines decision variables:**
  - `X(x_dim, N+1)`: State trajectory (scaled for numerical stability)
    ```
    X = diag([1, 1, 2, 0.01, ...]) * opti.variable()
    ```
    Scaling factors improve convergence by normalizing variable magnitudes
  - `U(u_dim, N+1)`: Control trajectory (jerk u = s⃛)
  - `T`: Total trajectory duration (free variable to optimize)
- **Extracts state components:**
  - `s_o = X(1,:)`: Path parameter trajectory (N+1 points)
  - `s_dot_o = X(2,:)`: Path velocity
  - `s_ddot_o = X(3,:)`: Path acceleration
- **Defines cost function:**
  ```
  cost = T + 0.01 * sum(U²) / N
  ```
  - Primary objective: minimize T (time)
  - Secondary regularization: penalize control effort to smooth trajectory

**Why it's needed:**
This initializes the optimization framework. The scaled variables prevent numerical ill-conditioning in IPOPT's solver.

**Key variables:**
- `opti`: CasADi Opti object
- `X`: State decision variables (N+1 time points)
- `U`: Control decision variables
- `T`: Duration (optimization variable)
- `N = 150`: Number of shooting intervals
- `N_final`: Final control point index (N_final ≈ 0.8*N for early termination)

---

### Section 15: Dynamics Definition & RK4 Integration (Lines 269-333)

**What it does:**

**A. Define ODE (Lines 269-318):**
- Defines the **4D sloshing dynamics** as an autonomous ODE system:
  ```
  State x = [s, ṡ, s̈, x_slosh_1, y_slosh_1, ẋ_slosh_1, ẏ_slosh_1, ...]

  ẋ(1) = x(2)  [ṡ = s_dot]
  ẋ(2) = x(3)  [s̈ = s_ddot]
  ẋ(3) = u     [s⃛ = control input]
  ẋ(4:7) = [sloshing dynamics for container 1]
  ...
  ```

**Sloshing equation per container:**
  ```
  A_c * [ẍ_s, ÿ_s]ᵀ = [B_c_x, B_c_y]ᵀ
  ```
  where A_c is a 2×2 inertia-like matrix and B_c includes:
  - Centrifugal and Coriolis terms (rotation coupling)
  - Natural frequency and damping terms
  - Nonlinear amplitude-dependent stiffening (as term)
  - Tray acceleration projection into container frame

- Creates CasADi Function `x_dot_fun(x, u)` = dx/dt

**B. RK4 Integration (Lines 321-333):**
- Implements 4th-order Runge-Kutta integration:
  ```
  k1 = f(x, u)
  k2 = f(x + dt/2*k1, u)
  k3 = f(x + dt/2*k2, u)
  k4 = f(x + dt*k3, u)
  x_next = x + dt/6*(k1 + 2*k2 + 2*k3 + k4)
  ```
- Maps RK4 across all N shooting intervals
- Enforces **continuity constraints:**
  ```
  X(:, i+1) == RK4(X(:, i), U(:, i), T/N)
  ```
  This ensures state propagation follows dynamics

**Why it's needed:**
The dynamics equations couple path evolution with sloshing. RK4 discretization converts continuous OCP to finite-dimensional NLP that IPOPT can solve.

**Key variables:**
- `x_dot_fun`: CasADi Function for ODE right-hand side
- `RK4`: CasADi Function for single RK4 step
- `RK4_map`: Mapped version for all N intervals
- `x_next_mat`: Predicted next states (used for continuity constraints)

---

### Section 16: Constraint Mapping (Lines 336-338)

**What it does:**
- Executes `mapCasadiFunctions.m` to vectorize constraint evaluations
- Maps constraint functions across all N+1 time points using CasADi's `.map()` feature
- Creates mapped versions:
  - `nonLift_map, nonSlip_map, nonTip_map, nonTwist_map`
  - `q_dot_map`: Joint velocity mapping

**Why it's needed:**
CasADi's `.map()` efficiently evaluates the same function at all time steps, reducing computational overhead and improving solver performance.

---

### Section 17: Adding Constraints to Optimization (Lines 339-392)

**What it does:**

**A. Path parameter constraints (Lines 340-341):**
```matlab
opti.subject_to(0 <= s_o <= 1);      % Path bounds
opti.subject_to(0 <= s_dot_o);        % No backward motion
```

**B. Control input constraints (Line 343):**
```matlab
opti.subject_to(u_min <= U <= u_max);  % Jerk bounds
```

**C. Sloshing constraints (Lines 352-356):**
For each container, enforces two-phase sloshing limit:
```matlab
% Phase 1 (normal motion): s ∈ [0, N_final]
opti.subject_to(sqrt(x_slosh_i^2 + y_slosh_i^2) <= eta_lim * scale);

% Phase 2 (settling): s ∈ [N_final, 1]
opti.subject_to(sqrt(x_slosh_i^2 + y_slosh_i^2) <= 0.2 * eta_lim * scale);
```
Where scale = mf*R / (ξ²*h*m₁)

Boundary condition: Sloshing must settle to zero by s = 1.

**D. Initial and final state constraints (Lines 358-360):**
```matlab
opti.subject_to(X(:,1) == x_start_OCP);           % Start at s=0
opti.subject_to(X(1:3, N_final:N+1) == x_end_OCP(1:3)); % End at s=1
```

**E. Joint velocity constraints (Lines 363-365):**
```matlab
opti.subject_to(q_dot_min <= q_dot <= q_dot_max);
```
Enforced at all N+1 time points.

**F. Waiter problem constraints (Lines 368-383):**
```matlab
for i = 1:num_containers
    opti.subject_to(nonLift(i,:) <= 0);   % All time steps
    opti.subject_to(nonSlip(i,:) <= 0);
    opti.subject_to(nonTip(i,:) <= 0);
    opti.subject_to(nonTwist(i,:) <= 0);
end
```

**G. Minimum trajectory time (Line 388):**
```matlab
opti.subject_to(T >= 0.3);  % At least 0.3 seconds
```

**Why it's needed:**
These constraints enforce physical feasibility, safety, and problem-specific requirements. IPOPT respects all constraints during optimization.

**Key variables:**
- All constraint functions and inequality relationships

---

### Section 18: Initial Guess & Solver Setup (Lines 393-413)

**What it does:**
- Sets initial guess for `s_o`: linear interpolation from 0 to 1
  ```matlab
  opti.set_initial(s_o, linspace(0,1,N+1));
  ```
- Sets initial trajectory duration: T = 5 seconds
- Configures IPOPT solver:
  ```matlab
  p_opts = struct();  % IPOPT options
  s_opts = struct('print_level', 5);  % Full convergence reporting
  opti.solver('ipopt', p_opts, s_opts);
  ```
- Sets callback to plot s trajectory evolution during optimization
- **Solves the OCP:**
  ```matlab
  tic
  sol = opti.solve();
  t_calc = toc
  ```
- Sets `is_solved = true` on successful termination

**Why it's needed:**
Good initial guesses improve solver convergence. A callback visualizes optimization progress in real time.

**Key variables:**
- `sol`: Optimal solution object (contains all optimized trajectories)
- `t_calc`: Computation time for optimization
- `is_solved`: Success flag

---

### Section 19: Solution Extraction (Lines 416-471)

**What it does:**
Extracts optimized trajectories from solution object:

**Path evolution:**
- `s_sol, s_dot_sol, s_ddot_sol`: Optimal path parameter and derivatives

**Waiter constraints (verification):**
- `nonLift_sol, nonSlip_sol, nonTip_sol, nonTwist_sol`: Constraint values at all time steps

**Tray quantities:**
- `p07_sol, v07_sol, a07_sol`: Tray position, velocity, acceleration
- `w07_sol, w07_dot_sol`: Tray angular velocity and acceleration
- `R07_sol, T07_sol`: Rotation matrix and transformation

**Orientation:**
- `thz_sol, thzd_sol, thzdd_sol`: Yaw angle and derivatives

**End-effector quantities:**
- `p06_sol, v06_sol, a06_sol`: EE position, velocity, acceleration
- `w06_sol, w06_dot_sol`: EE angular velocity and acceleration
- `R06_sol`: EE rotation matrix

**Joint trajectory:**
- `q_sol`: Joint angles via IK from EE poses
- `q_dot_sol`: Joint velocities (from solution)
- `U_sol`: Control input (jerk)

**Time vector:**
- `tvec`: Time samples corresponding to trajectory points
- `N_final`: Actual control termination point

**Display results:**
- Prints minimum trajectory time at N_final

---

### Section 20: Sloshing Height Simulation (Lines 473-554)

**What it does:**
For each container, post-processes the optimized trajectory to compute **actual sloshing heights** via ODE integration.

**A. Acceleration time series (Lines 475-483):**
For each time point, computes container acceleration in the world frame accounting for:
- Tray translational acceleration (a07_sol)
- Tray rotation effects: α × r + ω × (ω × r)

**B. ODE integration (Line 492):**
- Calls `ode45` to solve the **nonlinear sloshing ODE** over [0, t_end]
- Uses custom function `odefunctionNL_MSD` that solves the 4D mass-spring-damper system:
  ```
  [ẋ_s, ẏ_s, ẍ_s, ÿ_s] = f(t, S, liquid_params, accelerations)
  ```
- Initial conditions: zero sloshing [0, 0, 0, 0]

**C. Height computation (Lines 493-495):**
- Extracts sloshing displacement from ODE solution: S(:,1:2) = [x_s, y_s]
- Computes normalized height:
  ```
  η = (ξ²*h*m₁ / (m_f*R)) * sqrt(x_s² + y_s²)
  ```
- Stores results in variables `eta_1, eta_2, ...` (one per container)

**Why it's needed:**
The optimization uses a simplified 4D state model. ODE integration with the full nonlinear model provides accurate sloshing predictions for validation.

**Key variables:**
- `eta_i`: Sloshing height time series for container i
- `time_i`: Time vector for ODE solution
- `acc_x, acc_y, acc_z`: Container acceleration components
- `odefunctionNL_MSD`: Nonlinear ODE function handle

---

### Section 21: Sloshing Height Plots (Lines 502-554)

**What it does:**
- **Creates figure** showing η(t) for each container
- Plots actual sloshing heights vs. optimization time limit (η_lim)
- Plots a horizontal dashed line at `η_lim` for visual constraint verification
- Adds legends, labels, and formatting with LaTeX interpreter
- **Optionally saves:**
  - PDF figure to `Plots/`
  - MAT file with η time series to `Data/etas/`

**Why it's needed:**
Visual confirmation that sloshing limits are respected and safe margins exist.

**Key variables:**
- `fig_etas`: Figure handle
- `legend_entries`: Cell array of container names
- `save_name`: Output file name

---

### Section 22: Kinematic Magnitudes & Diagnostics (Lines 557-580)

**What it does:**
- Computes norms (magnitudes) of vector quantities:
  ```matlab
  a07_magnitude = sqrt(sum(a07_sol.^2, 1));  % Euclidean norm
  a06_magnitude = sqrt(sum(a06_sol.^2, 1));
  wd07_magnitude = sqrt(sum(w07_dot_sol.^2, 1));
  wd06_magnitude = sqrt(sum(w06_dot_sol.^2, 1));
  ```
- Finds peak values across trajectory
- Displays maximum accelerations and angular accelerations

**Why it's needed:**
Peak values help assess trajectory smoothness and feasibility for real robot execution.

---

### Section 23: Solver Diagnostics & Plotting (Lines 589-750)

**What it does:**
Runs several plotting/analysis scripts via `run()` command:

**`plotDiagnostics.m` (Line 590):**
- IPOPT convergence history
- Objective value evolution
- Constraint feasibility metrics
- Solver iteration statistics

**`plotSSolution.m` (Line 593):**
- Path parameter s(t), ṡ(t), s̈(t) vs. time
- Verifies s evolution is physically reasonable

**`plotEEQuantitiesSolutions.m` (Line 596):**
- End-effector position, velocity, acceleration
- EE rotation and angular velocity

**`plotTrayQuantitiesSolutions.m` (Line 599):**
- Tray position, velocity, acceleration
- Tray rotation (yaw angle and derivatives)

**`plotForcesSolutions.m` (Line 602, commented out):**
- Force and moment components at each container

**Waiter constraints plot (Lines 604-691):**
- 2×2 subplot showing nonLift, nonSlip, nonTip, nonTwist constraints
- All constraints must be ≤ 0 (plotted lines should stay below y=0 dashed line)
- Color-coded by container
- Saved as PDF if `save_fig = true`

**Joint solutions plot (Line 705):**
- Runs `plotJointSolutions.m`
- Shows joint angles q(t), velocities q̇(t), accelerations q̈(t)
- Compares optimized q (DK) vs. IK-computed q for verification

**Validation (Lines 715-731):**
- Checks joint velocities `q̇_max < robot.qp_max`
- Checks joint positions `robot.q_min < q < robot.q_max`
- Errors if any limit exceeded

**Collision check (Lines 739-747):**
- For every time step, calls `checkCollision()` on robot visualization
- Errors immediately if self-collision detected

**Summary plot (Line 750):**
- Runs `plotSummary.m` for overview visualization

**Why it's needed:**
Comprehensive post-optimization analysis ensures:
- Solver converged properly
- Solution satisfies all constraints
- Trajectory is executable on real robot

---

### Section 24: Joint Trajectory Validation & Export (Lines 752-775)

**What it does:**
- If `save_csv = true`:
  - **Resamples trajectory** from N+1 points to 500 Hz sampling
    ```matlab
    fHz = 500;  % Sampling frequency
    n = round(fHz * t_end + 1);  % Total samples
    ```
  - Uses `spline()` to interpolate q(t) on fine time grid
  - Adds initial joint configuration hold: `q_in` at 1 second
  - **Adds CSV offset:** User-configured offset to match robot home position
  - **Exports to CSV** in format expected by robot controller:
    ```
    Data/CSV/<figure_name>.csv
    ```
    Format: 6 rows (one per joint), columns are time points

**Why it's needed:**
Real robot execution requires:
- Smooth, densely-sampled trajectory at controller input frequency
- Time-indexed samples for synchronization
- Offset compensation for different reference frames

**Key variables:**
- `fHz = 500`: Output sampling rate
- `Tin = 1`: Initial hold duration (seconds)
- `tempo_spline`: High-resolution time vector
- `q_spline`: Interpolated joint angles
- `csv_joint_offset`: Offset added to all angles
- `q_tot_csv`: Final trajectory with offset

---

## Key Variables Reference

| Variable | Type | Meaning |
|----------|------|---------|
| `s` | scalar | Path parameter ∈ [0,1] |
| `s_dot` | scalar | Path parameter velocity |
| `s_ddot` | scalar | Path parameter acceleration |
| `u` | scalar | Control input (jerk = d³s/dt³) |
| `num_containers` | int | Number of containers on tray |
| `N` | int | Number of shooting intervals (150) |
| `N_final` | int | Termination point (≈ 0.8*N) |
| `T` | scalar | Total trajectory duration (optimization variable) |
| `X` | (x_dim, N+1) | State trajectory (scaled) |
| `U` | (u_dim, N+1) | Control trajectory |
| `x_dim` | int | State dimension = 3 + 4*num_containers |
| `u_dim` | int | Control dimension = 1 |
| `cyl_radius` | float | Container cylinder radius (m) |
| `fill_level` | float | Liquid fill height (m) |
| `eta_lim` | float | Sloshing limit (m) |
| `m_fluid` | float | Liquid mass (kg) |
| `wn` | float | Sloshing natural frequency (rad/s) |
| `zita` | float | Damping ratio |
| `p_7G_all` | (3, num_containers) | Container positions in tray frame |
| `p_06_sol` | (3, N+1) | End-effector position trajectory |
| `q_sol` | (6, N+1) | Joint angle trajectory |
| `q_dot_sol` | (6, N+1) | Joint velocity trajectory |
| `T07_sol` | (4, 4, N+1) | Tray transformation matrices |
| `nonLift_sol` | (num_containers, N+1) | Non-lift constraint values |
| `nonSlip_sol` | (num_containers, N+1) | Non-slip constraint values |
| `nonTip_sol` | (num_containers, N+1) | Non-tip constraint values |
| `nonTwist_sol` | (num_containers, N+1) | Non-twist constraint values |

---

## Execution Flow Diagram

```
START
  |
  v
[Load libraries & CasADi]
  |
  v
[Interactive Configuration - 5 Dialogs]
  +---> Container/liquid parameters
  +---> Container placement
  +---> Tray-to-EE transformation
  +---> Prescribed path definition
  +---> Orientation trajectory
  |
  v
[Kinematic Setup]
  +---> Robot model (SmartSix)
  +---> Transformation matrices
  +---> Inertia calculations
  |
  v
[Create CasADi Functions]
  +---> Trajectory: p(s), v(s,ṡ), a(s,ṡ,s̈)
  +---> Rotations: θ_z(s), ω(s,ṡ), α(s,ṡ,s̈)
  +---> Kinematics: q(s), q̇(s,ṡ), q̈(s,ṡ,s̈)
  +---> Constraints: nonLift, nonSlip, nonTip, nonTwist
  |
  v
[Optimal Control Problem (OCP)]
  +---> Decision variables: X, U, T
  +---> Cost function: minimize T + control_effort
  +---> Dynamics: RK4 integration of sloshing ODE
  +---> Constraints: bounds, continuity, waiter, sloshing
  |
  v
[Solve with IPOPT]
  +---> Multiple shooting discretization
  +---> Nonlinear programming
  +---> Convergence to feasible optimum
  |
  v
[Extract Solution]
  +---> Trajectory: s(t), q(t), p_EE(t)
  +---> Constraints verification
  +---> Peak values computation
  |
  v
[Post-Processing]
  +---> ODE integration for sloshing heights
  +---> IK verification
  +---> Collision checking
  |
  v
[Plotting & Analysis]
  +---> Solver diagnostics
  +---> Trajectory plots (s, q, EE, tray)
  +---> Sloshing height plots
  +---> Constraint margin visualization
  |
  v
[Optional Export]
  +---> CSV: Joint trajectory at 500 Hz
  +---> MAT: Sloshing time series
  +---> PDF: Figures
  |
  v
END
```

---

## Problem Formulation Summary

**Optimal Control Problem:**
```
minimize   T + 0.01 * ||U||²
subject to:
    Dynamics:    X_{k+1} = RK4(X_k, U_k, T/N)  ∀k
    Bounds:      0 ≤ s_o ≤ 1,  0 ≤ s_dot_o
    Kinematics:  q_dot_min ≤ q̇ ≤ q_dot_max  ∀t
    Waiter:      nonLift_i ≤ 0,  nonSlip_i ≤ 0,  nonTip_i ≤ 0,  nonTwist_i ≤ 0  ∀i,t
    Sloshing:    √(x_s^2 + y_s^2) ≤ η_lim  ∀i  (phase 1)
    Sloshing:    √(x_s^2 + y_s^2) ≤ 0.2*η_lim  ∀i  (phase 2)
    Initial:     X(:,1) = x_start,  X(1:3,N_final:end) = x_end
    Control:     u_min ≤ U ≤ u_max
    Time:        T ≥ 0.3
```

**State vector:**
```
X = [s, ṡ, s̈, x_s^(1), y_s^(1), ẋ_s^(1), ẏ_s^(1), ..., x_s^(num_containers), ...]
```

**Control vector:**
```
U = [s⃛] (jerk of path parameter)
```

**Constraint outputs (all must be ≤ 0):**
- `nonLift`: -F_z (normal force, must be compressive)
- `nonSlip`: F_x² + F_y² - μ²F_z² (friction violation)
- `nonTip`: F_x² + F_y² - (R/h_G)²F_z² (tipping violation)
- `nonTwist`: |M_z| - (2/3)μ|F_z|R (torque limit)

---

## Notes & Tips

1. **Initial Guess Sensitivity**: Poor initial guesses (especially for s_o) can cause IPOPT to diverge. The script uses linear interpolation as a safe default.

2. **Constraint Derivatives**: All constraint functions must have well-defined derivatives at the initial guess. Square roots of near-zero values cause numerical issues—the formulation avoids this.

3. **Scaling**: Decision variables are scaled by representative magnitudes. Without scaling, solver convergence is poor due to condition number issues.

4. **Time Discretization**: N=150 intervals balances accuracy and computational cost (~30-60 seconds per solve).

5. **Two-phase sloshing**: Early in trajectory (s < N_final ≈ 0.8), allow larger sloshing. Late in trajectory, force sloshing to settle (`0.2*eta_lim`).

6. **Mapped functions**: CasADi's `.map()` vectorizes function evaluation efficiently—never use loops for constraint evaluation.

7. **Post-optimization ODE**: The ODE integration (`ode45`) uses the nonlinear sloshing model, which is more accurate than the simplified 4D state used in optimization.

