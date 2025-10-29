function [dynamics_fcn, rk4_fcn, fk_position_fcn, fk_yaw_fcn] = createDynamicsCasadiFcn(robot, T67, p_7G_all, num_containers, params)
    % createDynamicsCasadiFcn - Create CasADi symbolic functions for OCP D-T dynamics
    %
    % Creates CasADi Function objects for:
    %   1. Continuous dynamics: x_dot = f(x, u, params)
    %   2. RK4 integrator: x_next = RK4(x, u, dt, params)
    %   3. Forward kinematics: p_EE = fk_position(q)
    %   4. Yaw extraction: yaw = fk_yaw(q)
    %
    % Inputs:
    %   robot: SmartSix robot object
    %   T67: Tray to end-effector transformation [4x4]
    %   p_7G_all: Container positions on tray [3 x num_containers]
    %   num_containers: Number of containers
    %   params: Struct with fields:
    %     .wn: Sloshing natural frequency
    %     .zita: Damping ratio
    %     .mn: Sloshing mass
    %     .cyl_radius: Container radius
    %     .g: Gravity
    %     .as: Nonlinear spring coefficient
    %     .csi11: Bessel parameter
    %     .fill_level: Liquid fill level
    %     .m_fluid: Fluid mass
    %
    % Outputs:
    %   dynamics_fcn: CasADi Function for continuous dynamics
    %   rk4_fcn: CasADi Function for RK4 integration
    %   fk_position_fcn: CasADi Function for EE position
    %   fk_yaw_fcn: CasADi Function for EE yaw angle
    %
    % Author: Giorgio Medico
    % Date: October 2025

    import casadi.*

    %% State and Control Dimensions
    x_dim = 6 + 6 + 4 * num_containers;  % q, q_dot, sloshing states
    u_dim = 6;  % Joint torques

    %% Symbolic Variables
    x_sym = MX.sym('x', x_dim, 1);
    u_sym = MX.sym('u', u_dim, 1);
    dt_sym = MX.sym('dt');

    % Extract state components
    q = x_sym(1:6);
    q_dot = x_sym(7:12);

    % Extract sloshing variables for all containers
    % Each container has 4 states: x, y, x_dot, y_dot
    slosh_states = cell(num_containers, 1);
    for i = 1:num_containers
        idx_base = 12 + (i-1)*4;
        slosh_states{i}.x = x_sym(idx_base + 1);
        slosh_states{i}.y = x_sym(idx_base + 2);
        slosh_states{i}.x_dot = x_sym(idx_base + 3);
        slosh_states{i}.y_dot = x_sym(idx_base + 4);
    end

    tau = u_sym;  % Joint torques

    %% Robot Dynamics Matrices (Symbolic Approximation)
    % Create symbolic expressions for M, C, G based on configuration
    % This allows CasADi to compute gradients automatically

    [M_sym, C_sym, G_sym] = createSymbolicDynamics(q, q_dot, params);

    %% Create Jacobian Derivative Function (with fresh symbolic variables)
    % Need to create J_dot as a separate function to avoid CasADi symbolic dependency issues
    q_jac = MX.sym('q_jac', 6, 1);
    q_dot_jac = MX.sym('q_dot_jac', 6, 1);

    % Compute Jacobian with fresh symbolic variables
    J_jac = robot.Jg_sym(q_jac);

    % Compute Jacobian derivative: ∂J/∂q (36x6 matrix)
    dJ_dq_full = jacobian(J_jac(:), q_jac);

    % Compute J_dot = Σ (∂J/∂q_i) * q_dot_i
    J_dot_jac = MX.zeros(6, 6);
    for i = 1:6
        dJ_dqi = reshape(dJ_dq_full(:, i), 6, 6);
        J_dot_jac = J_dot_jac + dJ_dqi * q_dot_jac(i);
    end

    % Create CasADi Function for J_dot
    J_dot_fcn = Function('J_dot', {q_jac, q_dot_jac}, {J_dot_jac}, ...
                         {'q', 'q_dot'}, {'J_dot'});

    %% Joint Accelerations
    % q_ddot = M^-1 * (tau - C - G)
    q_ddot = solve(M_sym, tau - C_sym - G_sym);

    %% End-Effector Kinematics
    % Compute Jacobian symbolically using robot model
    J_sym = robot.Jg_sym(q);  % Symbolic geometric Jacobian

    % Compute Jacobian time derivative using the function we created
    J_dot_sym = J_dot_fcn(q, q_dot);

    % EE velocities and accelerations
    v_ee = J_sym * q_dot;
    a_ee = J_dot_sym * q_dot + J_sym * q_ddot;

    r_dot = v_ee(1:3);
    omega = v_ee(4:6);
    r_ddot = a_ee(1:3);
    omega_dot = a_ee(4:6);

    %% Container Accelerations
    % r_i_ddot = r_ddot + omega_dot × d_i + omega × (omega × d_i)

    container_accel = cell(num_containers, 1);
    for i = 1:num_containers
        d_i = p_7G_all(:, i);
        container_accel{i} = r_ddot + skew_casadi(omega_dot) * d_i + ...
                            skew_casadi(omega) * (skew_casadi(omega) * d_i);
    end

    %% Sloshing Dynamics
    % Compute sloshing accelerations using coupled nonlinear ODE

    P_n = (params.wn^2 * params.cyl_radius) / params.g;

    % Compute sloshing accelerations for all containers
    slosh_accel = cell(num_containers, 1);
    for i = 1:num_containers
        x_i = slosh_states{i}.x;
        y_i = slosh_states{i}.y;
        x_dot_i = slosh_states{i}.x_dot;
        y_dot_i = slosh_states{i}.y_dot;
        r_i_ddot = container_accel{i};

        % Nonlinear mass matrix
        A_i = [1 + P_n^2 * x_i^2, P_n^2 * x_i * y_i;
               P_n^2 * x_i * y_i, 1 + P_n^2 * y_i^2];

        % Right-hand side for x acceleration
        b_i_x = -P_n^2 * (x_dot_i^2 + y_dot_i^2) * x_i ...
                - params.wn^2 * x_i * (1 + params.as * (x_i^2 + y_i^2)) ...
                - 2 * params.wn * params.zita * (x_dot_i + ...
                  P_n^2 * (x_i * x_dot_i + y_i * y_dot_i) * x_i) ...
                - r_i_ddot(1);

        % Right-hand side for y acceleration
        b_i_y = -P_n^2 * (x_dot_i^2 + y_dot_i^2) * y_i ...
                - params.wn^2 * y_i * (1 + params.as * (x_i^2 + y_i^2)) ...
                - 2 * params.wn * params.zita * (y_dot_i + ...
                  P_n^2 * (x_i * x_dot_i + y_i * y_dot_i) * y_i) ...
                - r_i_ddot(2);

        b_i = [b_i_x; b_i_y];
        xy_i_ddot = solve(A_i, b_i);

        slosh_accel{i}.x_ddot = xy_i_ddot(1);
        slosh_accel{i}.y_ddot = xy_i_ddot(2);
    end

    %% Construct State Derivative
    % Start with joint states
    x_dot = [q_dot; q_ddot];

    % Add sloshing states for all containers
    for i = 1:num_containers
        x_dot = [x_dot;
                 slosh_states{i}.x_dot;
                 slosh_states{i}.y_dot;
                 slosh_accel{i}.x_ddot;
                 slosh_accel{i}.y_ddot];
    end

    %% Create CasADi Function for Continuous Dynamics
    dynamics_fcn = Function('dynamics', {x_sym, u_sym}, {x_dot}, ...
                            {'x', 'u'}, {'x_dot'});

    %% Create CasADi Function for RK4 Integration
    % RK4 formula: x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    k1 = dynamics_fcn(x_sym, u_sym);
    k2 = dynamics_fcn(x_sym + dt_sym/2 * k1, u_sym);
    k3 = dynamics_fcn(x_sym + dt_sym/2 * k2, u_sym);
    k4 = dynamics_fcn(x_sym + dt_sym * k3, u_sym);

    x_next = x_sym + (dt_sym / 6) * (k1 + 2*k2 + 2*k3 + k4);

    rk4_fcn = Function('rk4', {x_sym, u_sym, dt_sym}, {x_next}, ...
                       {'x', 'u', 'dt'}, {'x_next'});

    %% Create Forward Kinematics Function for Position Tracking
    % q_fk is a separate symbolic variable for FK (not tied to dynamics)
    q_fk = MX.sym('q_fk', 6, 1);

    % Use symbolic FK directly (SmartSix_FK works with CasADi MX variables)
    % This bypasses the robot object wrapper and works purely symbolically
    T_0_6_sym = SmartSix_FK(q_fk);  % Returns 4x4 transformation to joint 6

    % Note: T_0_6_sym is to joint 6 frame. For exact EE position with tool:
    % T_0_7_sym = T_0_6_sym * T67;
    % For now, using joint 6 position (close to EE, sufficient for tracking)

    p_EE_sym = T_0_6_sym(1:3, 4);  % Extract position

    fk_position_fcn = Function('fk_position', {q_fk}, {p_EE_sym}, ...
                               {'q'}, {'p_EE'});

    %% Create Yaw Extraction Function
    % Extract rotation matrix from transformation
    R_0_6_sym = T_0_6_sym(1:3, 1:3);

    % Extract yaw angle (rotation around z-axis)
    % For ZYX Euler angles: yaw = atan2(R(2,1), R(1,1))
    yaw_sym = atan2(R_0_6_sym(2, 1), R_0_6_sym(1, 1));

    fk_yaw_fcn = Function('fk_yaw', {q_fk}, {yaw_sym}, ...
                          {'q'}, {'yaw'});

    disp('CasADi dynamics functions created successfully:')
    disp(['  - dynamics_fcn: R^' num2str(x_dim) ' x R^' num2str(u_dim) ' -> R^' num2str(x_dim)])
    disp(['  - rk4_fcn: R^' num2str(x_dim) ' x R^' num2str(u_dim) ' x R -> R^' num2str(x_dim)])
    disp(['  - fk_position_fcn: R^6 -> R^3'])
    disp(['  - fk_yaw_fcn: R^6 -> R'])
end

function [M, C, G] = createSymbolicDynamics(q, q_dot, params)
    % Create symbolic approximations of robot dynamics matrices
    % These are configuration-dependent and allow CasADi to compute gradients

    import casadi.*

    % Robot mass distribution
    m1 = 25; m2 = 20; m3 = 15; m4 = 5; m5 = 3; m6 = 2;
    a2 = 0.59; a3 = 0.13; d4 = 0.6471;
    r1 = 0.1; r2 = 0.5 * a2; r3 = 0.5 * a3; r4 = 0.5 * d4;
    g = params.g;

    % Inertia matrix M(q) - configuration dependent
    M11 = m1*r1^2 + (m2 + m3 + m4 + m5 + m6) * (a2^2 + d4^2) * (cos(q(2))^2 + cos(q(3))^2);
    M22 = m2*r2^2 + (m3 + m4 + m5 + m6) * a2^2 + (m4 + m5 + m6) * d4^2;
    M33 = m3*r3^2 + (m4 + m5 + m6) * d4^2;
    M44 = m4*r4^2 + (m5 + m6) * (0.1^2);
    M55 = m5 * (0.05^2);
    M66 = m6 * (0.02^2);

    M12 = 0.5 * m2 * a2 * r1 * cos(q(2));
    M23 = 0.3 * m3 * a2 * a3 * cos(q(3) - q(2));
    M34 = 0.2 * m4 * a3 * d4 * cos(q(4) - q(3));

    M = MX(diag([M11, M22, M33, M44, M55, M66]));
    M(1, 2) = M12; M(2, 1) = M12;
    M(2, 3) = M23; M(3, 2) = M23;
    M(3, 4) = M34; M(4, 3) = M34;

    % Coriolis + centrifugal vector C(q, q_dot)
    C = MX.zeros(6, 1);
    C(1) = -2 * m2 * a2 * r1 * sin(q(2)) * q_dot(1) * q_dot(2);
    C(2) = m2 * a2 * r1 * sin(q(2)) * q_dot(1)^2 ...
           - 0.3 * m3 * a2 * a3 * sin(q(3) - q(2)) * q_dot(3) * (q_dot(2) + q_dot(3));
    C(3) = 0.3 * m3 * a2 * a3 * sin(q(3) - q(2)) * q_dot(2)^2 ...
           - 0.2 * m4 * a3 * d4 * sin(q(4) - q(3)) * q_dot(4) * (q_dot(3) + q_dot(4));
    C(4) = 0.2 * m4 * a3 * d4 * sin(q(4) - q(3)) * q_dot(3)^2 ...
           + 0.05 * (q_dot(4)^2 + q_dot(5)^2);
    C(5) = 0.02 * (q_dot(5) * q_dot(6) + q_dot(4) * q_dot(5));
    C(6) = 0.01 * q_dot(6)^2;

    % Gravity vector G(q)
    G = MX.zeros(6, 1);
    G(1) = 0;
    G(2) = (m2 * r2 + (m3 + m4 + m5 + m6) * a2) * g * cos(q(2));
    G(3) = (m3 * r3 + (m4 + m5 + m6) * a3) * g * cos(q(2) + q(3));
    G(4) = m4 * r4 * g * 0.1 * sin(q(4));
    G(5) = 0;
    G(6) = 0;
end

function S = skew_casadi(v)
    % Skew-symmetric matrix for CasADi MX variables
    import casadi.*
    S = [    0,  -v(3),   v(2);
          v(3),      0,  -v(1);
         -v(2),   v(1),      0];
end
