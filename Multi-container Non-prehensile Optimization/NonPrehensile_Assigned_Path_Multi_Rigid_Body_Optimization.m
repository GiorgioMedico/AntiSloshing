%--------------------------------------------------------------------------
% Waiter problem multicontainers optimization
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it 
%    Date:       July 2025
%--------------------------------------------------------------------------

%% Load libraries
clear all
close all
clear classes
clc

win = 0;
if win
    addpath(genpath('Data\'));
    addpath(genpath('include\'));
    addpath(genpath("casadi-3.6.5-windows64-matlab2018b"));
else
    addpath(genpath('Data/'));
    addpath(genpath('include/'));
    addpath(genpath("casadi-3.6.5-linux64-matlab2018b"));
end
import casadi.*

%% Flags
[save_fig, save_csv, csv_joint_offset, animations] = getSimulationParametersTilt();
is_solved = false;

fig_name = "TO";
save_fig_folder = fullfile("Plots","Tilt");
URDF = 'comau_smartsix5.urdf.xacro';

%% Setup
% Interactive container parameters selection
[cyl_radius,cyl_height,total_mass,hG] = getAndVisualizeContainerParametersTilt(); % if windows do not close use "close all force"

g = 9.81;
% Inertia
Ixx = (1/12) * total_mass * (3 * cyl_radius^2 + cyl_height^2);
Iyy = Ixx;
Izz = 0.5 * total_mass * cyl_radius^2;
IG_G = diag([Ixx, Iyy, Izz]);

container.cyl_radius = cyl_radius;
container.cyl_height = cyl_height;
container.fill_level = cyl_height;
container.IG_G = IG_G;
container.total_mass = total_mass;
container.hG = hG; 
        

% Interactive tray and container placement selection
[num_containers, pMode, modeSpecificParam, tray_type, tLen, tWid, tThick, fCoeff,] = getContainerPlacementParameters();

tray = getTrayParameters(tray_type, tLen, tWid, tThick, fCoeff);

if strcmpi(pMode,"manual")
    outputVectors = get2DVectorInput(num_containers);
    [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container, pMode, outputVectors);
else
    [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container, pMode, modeSpecificParam);
end

R7G = eye(3);

% Display the displacement vectors
% disp('Diagonal displacement for each container:');
% disp(p_diag);

% Display the displacement vectors
disp('Displacement vectors for each container:');
disp(p_7G_all);

%% robot 
% Create Robot System Toolbox robot instance
% robot = create_SmartSix();
robot = SmartSix();

% Define Offset from end-effector frame to tray frame
% toolOffset  = [0 0 0.056]';
% toolRotationOffset = pi/4;
[toolOffset, toolRotationOffset] = getToolOffsetParameters();

R67 = Rz(toolRotationOffset);

p_67_6 = [0;0;tray.thickness] + toolOffset;
T67 = [R67 p_67_6;
        0 0 0 1];

% Safety factor for joint velocities
safety = 1;

% Define Inertia Matrix in end-effector and tray frames
IG_7 = R7G*container.IG_G*R7G';
IG_6 = R67*IG_7*R67';

% Visualize the Comau robot with tray and container setup
% run('visualizeSetup.m');
robot_visu = visualizeSetup(URDF,tray_type, tray, container, toolOffset, toolRotationOffset, num_containers, p_7G_all);


%% define robot limits
s_min     = 0;      s_max     = 1;
% s_dot_min   = 0;       s_dot_max   = 40;
% s_ddot_min  = -200;    s_ddot_max  = 200; 

s_j_lim = 1000;                                                           
th_j_lim = 1000;                                                           

u_min = [-s_j_lim,-th_j_lim,-th_j_lim]';  u_max = -u_min;        

q_min = robot.q_min';    q_max = robot.q_max';
q_dot_min = safety*robot.qp_min';    q_dot_max = safety*robot.qp_max';
q_ddot_min = safety*robot.qpp_min';    q_ddot_max = safety*robot.qpp_max';

%% Trajectory
[path_type, mode, p0Val, thz_0, thz_end, cpx, cpy, cpz, n_ctrl_pts] = getGeometricPathParameters(); % mode and p0Val are just debug variables, path_type is the name of the curve
fig_name = strcat(fig_name, "_", path_type);

fig_name = strcat(fig_name,"_", string(num_containers),"containers");

% Formulate pos, vel and acc of tray wrt base as Casadi functions of s and
% its derivatives
run("formulateTrajectory.m");

% plot the geometric path
% plotTrajectory(get_p_07_0);

%% Angular Trajectory
% Formulate z rotation and derivatives as Casadi functions of s and its
% derivatives
fig_name = strcat(fig_name,"_", string(rad2deg(thz_end)),"_deg");

% Define angle and its derivatives
run("formulateAngularTrajectory.m"); % linear dependance on s

% Define rotation matrix and angular vel and acc
run("formulateAngularQuantitiesTilt.m");

%% compute pos, vel and acc of ee 
run("formulateEndEffectorKinematicsTilt.m");

%% Differential Kinematics
% Create function of q and q_dot as function of s and s_dot
run("formulateDiffKinDepTilt.m");

%% Trajectory Preview and Sanity Check
if animations
    run("plotTrajPreviewAnimationTilt.m");
end

%% define initial and final conditions
x_dim = 9;
u_dim = 3;

x_start_OCP = zeros(x_dim,1);
x_end_OCP = zeros(x_dim,1);
x_end_OCP(1) = 1;

% Initial conditions
q_0 = full(get_q(0,0,0));
q_start = wrapToPi(q_0);

% Final conditions
q_fin = full(get_q(1,0,0));
q_end = wrapToPi(q_fin);

T_start = [ full(get_R06(0,0,0))    full(get_p_06_0(0,0,0));
                 0 0 0                1           ];

T_end = [ full(get_R06(1,0,0))    full(get_p_06_0(1,0,0));
                 0 0 0                1           ];

% plot init and final config
plotInitialFinalJointConfigs(robot_visu, "on", q_start, q_end, get_p_07_0, T_start, T_end, T67);

% % IMPORTANT: it is FUNDAMENTAL for optimization that the derivative of the
% % constraint functions is not ill-defined for the initial guess
% % eg. d/dx(sqrt(x))=1/(2*sqrt(x)) that for x=0 is ill-defined 

%% force contraints
s       = casadi.MX.sym('s', 1);         % position
s_dot   = casadi.MX.sym('s_dot', 1);     % velocity
s_ddot  = casadi.MX.sym('s_ddot', 1);    % acceleration

thx = casadi.MX.sym('thx',1);
thy = casadi.MX.sym('thy',1);
thx_dot = casadi.MX.sym('thx_dot',1);
thy_dot = casadi.MX.sym('thy_dot',1);
thx_ddot = casadi.MX.sym('thx_ddot',1);
thy_ddot = casadi.MX.sym('thy_ddot',1);

a_07_0 = get_a_07_0(s,s_dot,s_ddot);
R07 = get_R07(s,thx,thy);

xdd_0 = a_07_0;

w = get_w_0(s,s_dot,thx,thy,thx_dot,thy_dot);
alfa = get_w_dot_0(s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot);

w_7 = R07' * w;
w_dot_7 = R07' * alfa;
nonLift = [];
nonSlip = [];
nonTip = [];
nonTwist = [];
for i=1:num_containers
    xdd = xdd_0 + skew(alfa)*(R07*p_7G_all(:,i)) + skew(w)*(skew(w)*(R07*p_7G_all(:,i)));

    % Gravity force acting on each container expressed in frame 0
    fg_0 = -container.total_mass * g * [0; 0; 1];
    fg_7 = R07' * fg_0;
    
    % Moment from angular motion (constant across containers)
    M_iG_7 = -IG_7 * w_dot_7 - skew(w_7) * (IG_7 * w_7);
    
    fi_0 = -container.total_mass * xdd;
    f_0 = fi_0 + fg_0;
    fi_7 = R07' * fi_0;
    % Total Force acting on each container expressed in tray frame
    f_7 = fi_7 + fg_7;

    nonLift = [nonLift; f_7(3);];
    nonSlip = [nonSlip; f_7(1)^2 + f_7(2)^2 - tray.mu^2 * f_7(3)^2;];
    nonTip = [nonTip; f_7(1)^2 + f_7(2)^2 - (container.cyl_radius / container.hG)^2 * f_7(3)^2;];


    M_Q_7 = M_iG_7 + skew([0; 0; container.hG]) * f_7;
    Mz_max = (2/3) * tray.mu * abs(f_7(3)) * container.cyl_radius;
    nonTwist = [nonTwist; abs(M_Q_7(3)) - Mz_max;];

end

get_nonLift = casadi.Function('get_nonLift',  {s, s_dot, s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {nonLift});
get_nonSlip = casadi.Function('get_nonSlip',  {s, s_dot, s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {nonSlip});
get_nonTip = casadi.Function('get_nonTip',  {s, s_dot, s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {nonTip});
get_nonTwist = casadi.Function('get_nonTwist',  {s, s_dot, s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {nonTwist});

%% Optimization Parameters
N = 150; % number of control intervals 

opti = casadi.Opti(); % Optimization problem


% State Variables (scaled to improve convergence rate)
X = repmat([1;1;2; 0.1*ones(6,1);],1,N+1).*opti.variable(x_dim,N+1);
% Path parameter and derivatives
s_o   = X(1,:);
s_dot_o = X(2,:);
s_ddot_o = X(3,:);
% euler angles
thx_o     = X(4,:);
thy_o     = X(5,:);
% euler angles derivatives
thx_dot_o    = X(6,:);
thy_dot_o    = X(7,:);
% euler angles second derivatives
thx_ddot_o   = X(8,:);
thy_ddot_o   = X(9,:);

U = repmat([100;100;100],1,N+1).*opti.variable(u_dim,N+1);  
T = opti.variable();

%% Cost Functional
cost1 = 1e-5;
cost2 = 1e-5;
% cost1 = 1e-2;
% cost2 = 0;

opti.minimize(T + cost1*sumsqr(U(1,:))/N + cost2*sumsqr(U(2,:))/N + cost2*sumsqr(U(3,:))/N);

%% Dynamical System
x = casadi.MX.sym('x', x_dim);
u = casadi.MX.sym('u', u_dim);

f =  [x(2);x(3);u(1); %s
      x(6);x(7); %th
      x(8);x(9); %thd
      u(2);u(3);]; %thdd

x_dot_fun = casadi.Function('x_dot_fun', {x,u}, {f});

x = casadi.MX.sym('x', x_dim);
u = casadi.MX.sym('u', u_dim);
dt = casadi.MX.sym('dt');
k1 = x_dot_fun(x,         u);
k2 = x_dot_fun(x+dt/2*k1, u);
k3 = x_dot_fun(x+dt/2*k2, u);
k4 = x_dot_fun(x+dt*k3,   u);
x_next = x + dt/6*(k1+2*k2+2*k3+k4);
RK4 = casadi.Function('RK4', {x,u,dt}, {x_next});

RK4_map = RK4.map(N);
x_next_mat = RK4_map(X(:,1:N), U(:,1:N), T/N);
opti.subject_to(x_next_mat==X(:,2:N+1)); % close the gaps


%% Map constraint functions
run("mapCasadiFunctionsTilt.m");

%% constraint
opti.subject_to(0 <= s_o <= 1);             %s limits
opti.subject_to(0 <= s_dot_o);              %s dot limits

opti.subject_to(u_min <= U <= u_max);         % control jerk (input)

% Initial and Final Constraints
opti.subject_to(X(:,1) == x_start_OCP);
opti.subject_to(X(:,N+1) == x_end_OCP);

% Joint Velocity Constraints
q_dot_map = get_q_dot.map(N+1);
q_dot = q_dot_map(s_o,s_dot_o,thx_o,thy_o,thx_dot_o,thy_dot_o);
opti.subject_to(q_dot_min             <= q_dot        <= q_dot_max);

% Waiter Problem Constraints
nonLift_map = get_nonLift.map(N+1);
nonSlip_map = get_nonSlip.map(N+1);
nonTip_map = get_nonTip.map(N+1);
nonTwist_map = get_nonTwist.map(N+1);

nonLift = nonLift_map(s_o,s_dot_o,s_ddot_o,thx_o,thy_o,thx_dot_o,thy_dot_o,thx_ddot_o,thy_ddot_o);
nonSlip = nonSlip_map(s_o,s_dot_o,s_ddot_o,thx_o,thy_o,thx_dot_o,thy_dot_o,thx_ddot_o,thy_ddot_o);
nonTip = nonTip_map(s_o,s_dot_o,s_ddot_o,thx_o,thy_o,thx_dot_o,thy_dot_o,thx_ddot_o,thy_ddot_o);
nonTwist = nonTwist_map(s_o,s_dot_o,s_ddot_o,thx_o,thy_o,thx_dot_o,thy_dot_o,thx_ddot_o,thy_ddot_o);

for i=1:num_containers
    opti.subject_to(nonLift(i,:) <= 0);                                             
    opti.subject_to(nonSlip(i,:) <= 0);
    opti.subject_to(nonTip(i,:) <= 0);    
    opti.subject_to(nonTwist(i,:) <= 0);
end


%%%%%%%%%%%%%%%%%%%% CHECK NaN: opti.debug.value(jacobian(nonSlip,X),opti.initial())

opti.subject_to(T >= 0.3);  

% opti.subject_to(det>0.03);
% opti.subject_to(abs(det)>0.01);

%% initial solver's guess 
opti.set_initial(s_o, linspace(0,1,N+1));
opti.set_initial(T, 5);

%% Solve OCP
p_opts = struct();
s_opts = struct('print_level', 5); % 1 für nix % 5 für alles
opti.solver('ipopt',p_opts,s_opts); % set numerical backend

f_opti = figure();
set(f_opti, 'Name', 's Value Iteration', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 480]);

% hold on
opti.callback(@(i) plot(opti.debug.value(s_o)))

tic
sol = opti.solve();   % actual solve
t_calc = toc

is_solved = true;
% opti.debug.show_infeasibilities()


%% Solutions 
if ~is_solved
    sol = opti.debug();
    sol.show_infeasibilities(1e-4);
end

t_end = sol.value(T);
tvec = linspace(0,t_end,N+1);

% S
X_sol = sol.value(X);
s_sol = sol.value(X(1,:));
s_dot_sol = sol.value(X(2,:));
s_ddot_sol = sol.value(X(3,:));

thx_sol = sol.value(X(4,:));
thy_sol = sol.value(X(5,:));
thx_dot_sol = sol.value(X(6,:));
thy_dot_sol = sol.value(X(7,:));
thx_ddot_sol = sol.value(X(8,:));
thy_ddot_sol = sol.value(X(9,:));

% Waiter Problem
nonLift_sol = sol.value(nonLift);
nonSlip_sol = sol.value(nonSlip);
nonTip_sol = sol.value(nonTip);
nonTwist_sol = sol.value(nonTwist);

% Tray quantities
w07_sol = sol.value(w07);
w07_dot_sol = sol.value(w07_dot);
p07_sol = sol.value(p07);
v07_sol = sol.value(v07);
a07_sol = sol.value(a07);
R07_sol = sol.value(R07);
T07_sol = sol.value(get_T07(s_sol,thx_sol,thy_sol));

% Rotation
thz_sol = sol.value(thz);
thzd_sol = sol.value(thzd);
thzdd_sol = sol.value(thzdd);

% EE quatities
w06_sol = sol.value(w06_0);
w06_dot_sol = sol.value(w06_0_dot);
p06_sol = sol.value(p06);
v06_sol = sol.value(v06);
a06_sol = sol.value(a06);
R06_sol = sol.value(R06);

% Joints
q_dot_sol = sol.value(q_dot);
q_sol = full(get_q(s_sol,thx_sol,thy_sol));
q_sol = unwrap(q_sol, [], 2);

% Control Input
U_sol = sol.value(U)';
U_sol_end = [U_sol;U_sol(end,:)]';

disp(' ')
disp('--------------------------')
disp(['minimum time is: ', num2str(tvec(N+1))])
disp('--------------------------')

%% Magnitudes
% Compute acceleration magnitude
a07_magnitude = sqrt(sum(a07_sol.^2, 1)); % Norm along rows (3 components)
a06_magnitude = sqrt(sum(a06_sol.^2, 1)); % Norm along rows (3 components)
wd07_magnitude = sqrt(sum(w07_dot_sol.^2, 1)); % Norm along rows (3 components)
wd06_magnitude = sqrt(sum(w06_dot_sol.^2, 1)); % Norm along rows (3 components)
v07_magnitude = sqrt(sum(v07_sol.^2, 1)); % Norm along rows (3 components)

% Find the maximum acceleration value
max_a07 = max(a07_magnitude);
max_a06 = max(a06_magnitude);
max_wd07 = max(wd07_magnitude);
max_wd06 = max(wd06_magnitude);

max_v07 = max(v07_magnitude);
max_thzdd = max(thzdd_sol);
max_thxdd = max(thx_ddot_sol);
max_thydd = max(thy_ddot_sol);

% Display the result
disp(['Maximum tray acceleration is: ', num2str(max_a07)]);
disp(['Maximum ee acceleration is: ', num2str(max_a06)]);

disp(['minimum tray angular acceleration is: ', num2str(max_wd07)])
disp(['minimum ee angular acceleration is: ', num2str(max_wd06)])


%% Warm restart 
% % opti.subject_to(nonSlip <= 0);
% % opti.subject_to(nonTip <= 0);   
% opti.set_initial(sol.value_variables()); 
% sol2 = opti.solve(); 
% sol2.stats.iter_count

%% Solver Diagnostics 
run("plotDiagnostics.m");

%% s and derivatives
run("plotSSolution.m");

%% End Effector
run("plotEEQuantitiesSolutions.m");

%% Tray
run("plotTrayQuantitiesSolutions.m");

%% Forces
% run("plotForcesSolutions.m");
% colors =  jet(num_containers);
fig_waiter = figure();
set(gcf, 'Name', 'Waiter Constraints', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);
subplot(2,2,1)
hold on
grid on
box on
for i = 1:num_containers
    plot(tvec,nonLift_sol(i,:),'LineWidth',1.5);
    
    legend_entries{i} = sprintf('$C_{%d}$', i);
end
yline(0, '--k', 'LineWidth', 1.2)
xlim([0, tvec(end)])
title('Non Lift', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$F$ [N]', 'Interpreter', 'latex')
lgd = legend(legend_entries, 'Interpreter', 'latex', 'Location', 'northeast');
set(lgd, 'ItemTokenSize', [10, 10]);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

subplot(2,2,2)
% set(gcf, 'Name', 'nonSlip_sol', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);
hold on
grid on
box on
for i = 1:num_containers
    plot(tvec,nonSlip_sol(i,:),'LineWidth',1.5);
    
    legend_entries{i} = sprintf('$C_{%d}$', i);
end
yline(0, '--k', 'LineWidth', 1.2)
xlim([0, tvec(end)])
title('Non Slip', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$F$ [N]', 'Interpreter', 'latex')
lgd = legend(legend_entries, 'Interpreter', 'latex', 'Location', 'northeast');
set(lgd, 'ItemTokenSize', [10, 10]);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

subplot(2,2,3)
% set(gcf, 'Name', 'NonTip', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);
hold on
grid on
box on
for i = 1:num_containers
    plot(tvec,nonTip_sol(i,:),'LineWidth',1.5);
    
    legend_entries{i} = sprintf('$C_{%d}$', i);
end
yline(0, '--k', 'LineWidth', 1.2)
xlim([0, tvec(end)])
title('Non Tip Over', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$T$ [Nm]', 'Interpreter', 'latex')
lgd = legend(legend_entries, 'Interpreter', 'latex', 'Location', 'northeast');
set(lgd, 'ItemTokenSize', [10, 10]);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

subplot(2,2,4)
% set(gcf, 'Name', 'NonTwist', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);
hold on
grid on
box on
for i = 1:num_containers
    plot(tvec,nonTwist_sol(i,:),'LineWidth',1.5);
    
    legend_entries{i} = sprintf('$C_{%d}$', i);
end
yline(0, '--k', 'LineWidth', 1.2)
xlim([0, tvec(end)])
title('Non Twist', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$T$ [Nm]', 'Interpreter', 'latex')
lgd = legend(legend_entries, 'Interpreter', 'latex', 'Location', 'northeast');
set(lgd, 'ItemTokenSize', [10, 10]);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
if save_fig
    save_name = strcat(fig_name,'_waiter.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(fig_waiter, 'Renderer', 'painters')  % Forces vector output where possible
    set(fig_waiter, 'Units', 'Inches');
    fig_pos = get(fig_waiter, 'Position');
    set(fig_waiter, 'PaperUnits', 'Inches');
    set(fig_waiter, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(fig_waiter, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    
    print(fig_waiter, '-dpdf', save_name)
end

%% Joint
% Check difference between IK (post-proc) and DK (opti) solutions
q_eval = zeros(6,N+1);
for i = 1:N+1
    T_ = [ R06_sol(:,3*(i-1)+1:3*i)   p06_sol(:,i);
            0 0 0    1  ];
    % q_eval(:,i) = SmartSix_IK_(robot,T_,0);
    q_eval(:,i) = robot.ik(T_,0);
end
% q_eval = unwrap(q_eval);
q_eval = unwrap(q_eval, [], 2);

run("plotJointSolutions.m");

% figure()
% title("Difference between optimization joints and ik joints")
% hold on
% grid on
% box on
% plot(tvec,q_sol)
% plot(tvec,q_eval,'k--')

qp_max = [max(abs(q_dot_sol(1,:))),max(abs(q_dot_sol(2,:))),max(abs(q_dot_sol(3,:))),max(abs(q_dot_sol(4,:))),max(abs(q_dot_sol(5,:))),max(abs(q_dot_sol(6,:)))];

if qp_max < robot.qp_max
    disp('q_dot are OK!')
else
    error('q_dot surpass the maximum values')
end

q = q_eval;
q_max = [max(q(1,:)),max(q(2,:)),max(q(3,:)),max(q(4,:)),max(q(5,:)),max(q(6,:))];
q_min = [min(q(1,:)),min(q(2,:)),min(q(3,:)),min(q(4,:)),min(q(5,:)),min(q(6,:))];

if all(q_min > robot.q_min) && all(q_max < robot.q_max)
    disp('q are OK!')
else
    disp('q surpass the maximum values')
end

%% Optimized Trajectory Animation
if animations
    run("plotTrajExecAnimation.m");
end

%% Check no collision
for i = 1:N+1
    % Check for self-collisions
    collisionResults = checkCollision(robot_visu,q_eval(:,i)',"SkippedSelfCollisions",'parent');
    
    % Display the result of the collision check
    if any(collisionResults)
        error('Self-collision detected in the robot at time step %d!\n', i);
    end
end

%% Final Plot 
run("plotSummaryTilt.m");

%% Save joint trajectory
fHz = 500;
Tin     = 1;
n_in    = Tin*fHz + 1;

if save_csv
    % dt_ = 1 / fHz;  % Sampling interval for 500 Hz
    % t_new = 0:dt_:t_end;
    
    tempo_finale = tvec(N);
    tempo = linspace(0,tempo_finale,N);
    n = round(fHz*tempo_finale + 1);
    tempo_spline = linspace(0,tempo_finale,n);

    q_spline = spline(tempo,q_sol(:,1:N),tempo_spline);
    q_in = zeros(6,n_in);
    for i = 1:6
        q_in(i,:) = q_spline(i,1)*ones(1,n_in);
    end
    q_tot_csv  = [q_in q_spline(:,2:end)] + csv_joint_offset; 
        
    % Save the data to a CSV file
    writematrix(q_tot_csv', fullfile('Data','CSV','Tilt',strcat(fig_name,'.csv')), 'Delimiter', ';');
end
