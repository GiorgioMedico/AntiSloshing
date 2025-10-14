%--------------------------------------------------------------------------
% Trajectory Planning for MATRIX project (Comau SmartSix robot) - Scara
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it 
%    Date:       January 2025
%--------------------------------------------------------------------------
clear all
close all
clc

%% import 
ws_path = fullfile('..', '..');
kin_path = fullfile(ws_path,"Kinematics");
odes_path = fullfile(ws_path,"Sloshing_model", "odes");
traj_path = fullfile(ws_path,"Control", "Algebraic_Control", "Slosh", "Single_container", "utils");

% add folder and subfolders
addpath(genpath(kin_path));
addpath(genpath(odes_path));

addpath(genpath('utils\'));
addpath(genpath('Data\'));
addpath(genpath(traj_path));


%%
robot       = create_SmartSix();
% jointOffset = [0, 0, pi/2, 0, 0, 0];
toolOffset  = [0 0 0.056]';
tray_thickness = [0 0 0.01]';
toolOffset  = toolOffset + tray_thickness;

% toolOffset  = [0 0 0.28]';

robot_visu = importrobot('BACKUP_comau_smartsix5.urdf.xacro');
robot_visu.DataFormat = 'row'; % Options: 'row', 'column', or 'struct'

path_type   = 'y';
dim_type    = '2D';

% set rotation about z axis (float-rad)
rot = pi/2;

motion_type = [path_type,'_',dim_type];
Ts = 1;
% BuildTrajectoryGenerator([0.5 5/Ts 10/Ts^2 60/Ts^3 200/Ts^4],[18.9612],0.002)
% Te          = 20;
% Te = 1.2;
Dz_max      = 0.6;

save_video = 0;
save_csv   = 1;
save_fig   = 0;

%% Parameters
R = 0.049;
h = 0.08;
[g, rho, ~, V, csi11, Cs, ms, ~, cs, as, ~, ~, ~, wn] = Parameters(R, h);
ln = g/wn^2;

hn = 1/2*h*(1 - 4*R/(csi11*h)*tanh(csi11*h/(2*R)));

% freq = 500;
% n    = freq*Te + 1;
% time = linspace(0,Te,n);
% [sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

%% Trajectory
freq = 500;
compl_traj = 0;
if compl_traj
[rEx, rEdx, rEddx, rEdddx, rEddddx, rEy, rEdy, rEddy, rEdddy, rEddddy, rEz, rEdz, rEddz, rEdddz, rEddddz, th, thd, thdd, dist, Te, time, n] = FIR_traj(path_type, dim_type, rot, robot, Ts);
Deltath = max(th)-min(th);
%%
else
% Te = 1.8; % Total time [s]
% n = freq * Te + 1;
% time = linspace(0, Te, n)';
% dist = 0;
% 
% % Time splitting
% T1 = Te / 2; % Translation duration
% T2 = Te / 2; % Rotation duration
% n1 = round(T1 * freq);
% n2 = n - n1;

T1 = 0.8; % Translation duration [s]
T2 = 1.0; % Rotation duration [s]

% Total time derived from T1 and T2
Te = T1 + T2;

% Time vector and number of points
n = round(freq * Te) + 1;
time = linspace(0, Te, n)';
dist = 0;

% Number of steps for each phase
n1 = round(T1 * freq);
n2 = n - n1;

% --- First phase: translation along y-axis ---
t1 = linspace(0, T1, n1);
[sigma1, sigmad1, sigmadd1] = motion_law(0, 1, 0, 0, t1);

% Displacement in y of 0.5 m
d = 0.5;

rEx = zeros(1, n);
rEdx = zeros(1, n);
rEddx = zeros(1, n);

rEy = [d * sigma1, d * ones(1, n2)];
rEdy = [d * sigmad1, zeros(1, n2)];
rEddy = [d * sigmadd1, zeros(1, n2)];

rEz = zeros(1, n);
rEdz = zeros(1, n);
rEddz = zeros(1, n);

% --- Second phase: rotation around z-axis ---
t2 = linspace(0, T2, n2);
[th2, thd2, thdd2] = motion_law(0, rot, 0, 0, t2);  % 180° rotation

th = [zeros(1, n1), th2];
thd = [zeros(1, n1), thd2];
thdd = [zeros(1, n1), thdd2];
end

fig_name = strcat('Data/290425/',motion_type,'_', num2str(dist),'m_',num2str(Te),'s_',num2str(rad2deg(rot)),'deg');

if rot ==0 
    fig_name = strcat(fig_name, '_noRot');
end
if toolOffset(3) >0.1 
    fig_name = strcat(fig_name, '_CoR');
end

figure()
subplot(1,3,1)
title("Pos")
hold on
grid on
plot(time, rEx);
plot(time, rEy);
plot(time, rEz);
legend("x", "y", "z")
subplot(1,3,2)
title("Vel")
hold on
grid on
plot(time, rEdx);
plot(time, rEdy);
plot(time, rEdz);
legend("x", "y", "z")
subplot(1,3,3)
title("Acc")
hold on
grid on
plot(time, rEddx);
plot(time, rEddy);
plot(time, rEddz);
legend("x", "y", "z")



%%

% initial configuration
% T06_0 = T(Rz(pi), [0.7458; -0.3153; 1.1146]);
% T06_0 = T(Rz(pi), [0.7458; -0.3153; 1.2146]);
T06_0 = T(Rz(pi), [0.7458; -0.0; 1.3146]);

qOffset = SmartSix_IK(robot, T06_0, 0);

% Set the robot configuration
figure()
show(robot_visu, qOffset, 'Frames', 'on', 'PreservePlot', true);
hold on
% Set view and labels for better visualization
title('Comau SmartSix Robot in Initial Joint Configuration');
xlabel('X');
ylabel('Y');
zlabel('Z');
% Set axes limits starting from 0
xlim([-1.5 1.5])
ylim([-1.5 1.5])
zlim([0 2])


thd_max = max(thd)
thdd_max = max(thdd)

cor = [0;0;h/2+hn];
% T_offset = SmartSix_FK(qOffset')*T(eye(3),toolOffset+cor)
T06_0 = SmartSix_FK(qOffset')

offset = toolOffset+cor; % {p67}0
T67 = T(eye(3), T06_0(1:3,1:3)'*offset);

% T07_0 = SmartSix_FK(qOffset')*T(eye(3),toolOffset+cor) % displacement on z axis
T07_0 = T06_0*T67;


rE   = [rEx; rEy; rEz];
rEd  = [rEdx; rEdy; rEdz];
rEdd = [rEddx; rEddy; rEddz];

y_max = max(rEdd(2,:))

% gamma = -atan2(rEdx,rEdy);
% for i=1:length(rEdx)
%     if isnan(rEdx(i)/rEdy(i)) 
%         % if i==length(rEdx)
%         if i>length(rEdx)/2
%             gamma(i) = pi/2;
%         end
%     end
% end
% figure
% plot(time, gamma)


if ~isempty(th)
    wE   = [zeros(1,n); zeros(1,n); thd];
else
    wE   = [zeros(1,n); zeros(1,n); zeros(1,n)];
end

%% compensation
phi = zeros(1,n);
phi_dot = zeros(1,n);
theta = zeros(1,n);
theta_dot = zeros(1,n);
theta_ddot = zeros(1,n);
phi_ddot = zeros(1,n);


figure()
title("Compensation angles")
hold on
plot(time, phi)
plot(time,theta)
% plot(time,th)
grid on
legend("$\phi$","$\theta$","$\gamma$","Interpreter", "latex")


figure()
title("Tilting compensation angle")
hold on
plot(time,theta)
plot(time,theta_dot)
% plot(time,th)
grid on
legend("$\theta$","$\dot\theta$","Interpreter", "latex")

%% Comau Inverse Kinematics

for i = 1:n
    
    vel_norm(i)   = norm(rEd(:,i));
    acc_norm(i)   = norm(rEdd(:,i));
    acc_norm2D(i) = norm(rEdd(1:2,i));

    if ~isempty(th)
        T_rot = T(Rz(th(i)), zeros(3,1));
    else
        T_rot = eye(4);
    end
    T_disp = T(eye(3), rE(:,i));

    T_phi = T(Rz(phi(i)), zeros(3,1));
    T_phi_minus = T(Rz(-phi(i)), zeros(3,1));
    T_theta = T(Ry(theta(i)), zeros(3,1));


    % T07 = T_disp*T07_0*T_rot;
    T07 = T_disp*T07_0*T_rot*T_phi*T_theta*T_phi_minus;
    % T07 = T_disp*T07_0*T_rot*T_phi*T_phi_minus;
    T07_t(:,:,i) = T07;
    T06 = T07*inv(T67);
    T06_t(:,:,i) = T06;
    q_sol(:,i) = SmartSix_IK(robot,T06,0);

    
    % T06 = [Rz(th(i)), rE(:,i);
    %        0   0   0     1   ];
    % T06 = [Rz(th(i)), T_offset(1:3,1:3)*rE(:,i);
    %        0   0   0     1   ];
    
    % T07_t(:,:,i) = T_offset*T06;
    % q_sol(:,i) = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
    Jg = SmartSix_Jg(q_sol(:,i));

    w7_7 = [0; 0; thd(i)] + T_phi(1:3,1:3) * [0; theta_dot(i); 0];
    R07 = T07(1:3,1:3);
    w7_0 = R07*wE(:,i);
    w7_0 = R07*w7_7;

    v06 = rEd(:,i) - skew(w7_0)*T06(1:3,1:3)*T67(1:3,4);

    qd(:,i) = Jg\[v06; w7_0];

    % qd(:,i) = Jg\[rEd(:,i) - skew(wE(:,i))*Rz(th(i))*toolOffset; wE(:,i)];

end
% T06 = [Rz(th(1)), rE(:,1);
%        0   0   0     1   ];
% T06 = [Rz(th(1)), T_offset(1:3,1:3)*rE(:,1);
%        0   0   0     1   ];
% q_0 = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
% rad2deg(q_0 + jointOffset)

q_0 = qOffset;

q = q_0' + cumtrapz(time,qd,2);

max(abs(vel_norm))

% q_sol(6,:)=q_sol(6,:)+qOffset(6);
q_sol = unwrap(q_sol, [], 2);
figure()
title("Joint computation comparison - ik vs dk")
hold on
plot(time,unwrap(q_sol, [], 2))
plot(time,q, "LineStyle","--","Color","black")
legend("q1","q2","q3","q4","q5","q6")

%% Animation
animations = 1;
x = zeros(1,n);
y = zeros(1,n);
z = zeros(1,n);
for i=1:n
    x(i)=T07_t(1,4,i);
    y(i)=T07_t(2,4,i);
    z(i)=T07_t(3,4,i);
end

if animations
    numSteps = n; % Number of steps in the trajectory
    jointTrajectory = zeros(6,n);
    for i=1:n
        jointTrajectory(:,i) = [q_sol(1,i), q_sol(2,i), q_sol(3,i), q_sol(4,i), q_sol(5,i), q_sol(6,i)]';
        % jointTrajectory(:,i) = [q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i)]';

    end
    
    if save_video
        % videoFile = 'robot_trajectory_animation.mp4'; % You can change the filename and format here
        videoFile = strcat(fig_name,'.mp4');
        v = VideoWriter(videoFile, 'MPEG-4'); % Using the MPEG-4 codec for the video
        open(v);
    end
    % Open a new figure for the animation
    fg = figure;
    fg.WindowState = 'maximized';
    % plot3(rE(1,:),rE(2,:),rE(3,:),'LineWidth',2)
    % plot3(TTT(1,4,:),TTT(2,4,:),TTT(3,4,:),'LineWidth',2)
    % plot3(T_07_0(1,4),T_07_0(2,4),T_07_0(3,4),'LineWidth',2, 'Marker','*','Color','b')
    plot3(x,y,z ,'LineWidth',2,'Marker','*','MarkerIndices',round(linspace(1, n, 3)))
    hold on;
    axis equal;
    view(3);
    % view([2 0 0]);
    % view([0 T06_0(1,4) 0]);
    title('Comau Robot Joint Trajectory');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    % Add lighting to the figure
    light('Position', [1 0 1], 'Style', 'infinite');
    % light('Position', [-1 -1 1], 'Style', 'infinite');
    
    % Optionally, add a light behind the robot for balanced lighting
    light('Position', [0 0 -1], 'Style', 'infinite');
    
    disp('Press any key in the plot window to start the animation...');
    waitforbuttonpress;
    
    
    % Set up rate control to control the animation speed
    timePerFrame = Te / n;
    % timePerFrame = 0.02;
    rate = 1/timePerFrame;
    rateCtrl = robotics.Rate(rate);  % Adjust rate for desired speed
    
    % Loop through each configuration in the joint trajectory
    for i = 1:10:numSteps
        % tic
        % Extract the current joint configuration
        currentConfig = jointTrajectory(:, i)';
        
        % Display the robot at the current configuration
        show(robot_visu, currentConfig, 'Frames', 'on', 'PreservePlot', false);
        
        if save_video
            % Capture the current frame of the figure
            frame = getframe(fg); % Capture the frame from the figure window
            
            % Write the frame to the video
            writeVideo(v, frame);
        end
        
        % Wait for the next loop to maintain the rate
        waitfor(rateCtrl);
        % toc
    end
    
    if save_video
        close(v);
    end
end

%% Sloshing-Height Formulation
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);


S0 = [0 0];
% [tL,sL] = ode45(@(t,s)odeP_ZY(t,s,l,k,Cs,ms,time,rEddx,rEddy,rEddz,phi,J,g,as,wn,'L'), tspan, S0);

% [tL,sL] = ode45(@(t,s)odeP_algtilt(t,s,l,k,Cs,ms,time,rEddx,rEddy,rEddz,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,J,g,as,wn,'L'), tspan, S0);
% [tNL,sNL] = ode45(@(t,s)odePY(t,s,wn,Cs,g,time,rEddy,rEddz,'NL'), tspan, S0);

beta = zeros(3,n);
beta_dot = zeros(3,n);
beta_ddot = zeros(3,n);
for i=1:n
    beta(:,i) = Rz(phi(i))*[0;theta(i);0];
    beta_dot(:,i) = Rz(phi(i))*[0;theta_dot(i);0];
    beta_ddot(:,i) = Rz(phi(i))*[0;theta_ddot(i);0];
end

figure
title("Tilting angle in model comparison")
hold on 
plot(time,beta(1,:))
% plot(time,beta_dot(1,:))
% plot(time,beta_ddot)
plot(time,theta)
% plot(time,theta_dot)
% plot(time,theta_ddot)
grid on

[tNL,sNL] = ode45(@(t,s)odePY_tiltY(t,s,wn,Cs,g,ln,time,rEddy,rEddz,-beta(1,:),-beta_dot(1,:),-beta_ddot(1,:),'NL'), tspan, S0);

figure
hold on
title("Slosh angle NL model")
plot(tNL,sNL)
grid on

etaPNL_NL = zeros(length(tNL),1);
for i=1:length(tNL)
    etaPNL_NL(i) = abs(R*tan(sNL(i)));
end

etaNLPar = etaPNL_NL;


S0 = [0 0];
% [tL,sL] = ode45(@(t,s)odeP_ZY(t,s,l,k,Cs,ms,time,rEddx,rEddy,rEddz,phi,J,g,as,wn,'L'), tspan, S0);

% Set custom tolerances
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

% [tL,sL] = ode45(@(t,s)odeP_algtilt(t,s,l,k,Cs,ms,time,rEddx,rEddy,rEddz,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,J,g,as,wn,'L'), tspan, S0);
[tL,sL] = ode45(@(t,s)odePY_tiltY(t,s,wn,Cs,g,ln,time,rEddy,rEddz,-beta(1,:),-beta_dot(1,:),-beta_ddot(1,:),'L'), tspan, S0, options);

figure
hold on
title("Slosh angle L model")
plot(tL,sL)
grid on

etaPNL_L = zeros(length(tL),1);
for i=1:length(tL)
    etaPNL_L(i) = abs(R*tan(sL(i)));
end

etaLPar = etaPNL_L;



% figure()
% plot(tL,etaL)

%% Graphics
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 3.5;
num_cols    = 3;

max_slosh = max(max(etaNLPar*1000),max(etaLPar*1000));
Ylim      = (floor(max_slosh/5)+1)*5;


fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
subplot(2,3,2)
hold on
grid on
box on 
plot(time,rEddx,'LineWidth',line_width)
plot(time,rEddy,'LineWidth',line_width)
plot(time,rEddz,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('[m/s$^2$]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\ddot r_x$','$\ddot r_y$', '$\ddot r_z$', 'Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
grid on
box on 
plot(tL, etaLPar*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
plot(tNL, etaNLPar*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
legend('L Model', 'NL Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,1)
hold on
grid on
box on 
if ~isempty(th)
    plot(time,th,'LineWidth',line_width)
    plot(time,thd,'LineWidth',line_width)
    plot(time,thdd,'LineWidth',line_width)
    xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
    xlim([0 Te])
    legend('$\theta$ [rad]','$\dot \theta$ [rad/s]', '$\ddot \theta$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
    set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
end
subplot(2,3,4)
hold on
grid on
box on
% plot(time,q./robot.q_max','LineWidth',line_width)
% plot(time,q_sol./robot.q_max','--k')
plot(time,q,'LineWidth',line_width)
plot(time,q_sol,'--k')
xlabel('t [s]','interpreter','latex','FontSize',label_size)
ylabel('${q}_{i}$','interpreter','latex','FontSize',label_size)
legend('1','2','3','4','5','6','interpreter','latex','FontSize',10)
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,5)
hold on
grid on
box on
plot(time,qd./robot.qp_max','LineWidth',line_width)
xlabel('t [s]','interpreter','latex','FontSize',label_size)
ylabel('$\dot{q}_{i}/\dot{q}_{i,max}$','interpreter','latex','FontSize',label_size)
line([0 Te],[1 1],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
line([0 Te],[-1 -1],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
legend('1','2','3','4','5','6','interpreter','latex','FontSize',10)
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,6)
hold on
box on 
xlim([0 5])
ylim([0 5])
text(0.5,4,['$||\ddot{\mathbf{r}}||_{max} = $',num2str(max(acc_norm)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,3.5,['$||\ddot{\mathbf{r}}_{2D}||_{max} = $',num2str(max(acc_norm2D)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,3,['$|\ddot{r}_z|_{max} = $',num2str(max((abs(rEddz)))),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,2.5,['$|\ddot{\theta}|_{max} = $',num2str(max((abs(thdd)))),'[rad/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,2.0,['$\overline \eta_{L,max} = $',num2str(max(etaLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,1.5,['$\overline \eta_{NL,max} = $',num2str(max(etaNLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,1.0,['$T_{e} = $',num2str(Te),'[s]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,0.5,fig_name,'FontSize', label_size, 'Interpreter', 'latex')
if save_fig
    save_name = strcat(fig_name,'.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
%     save_name = strcat(fig_name,'.pdf');
%     print('-dpdf', '-fillpage', save_name)
end



figure()
title("Geometric path")
hold on
grid on
box on
axis equal
view(3)
% view([90 0])
plot3(rEx,rEy,rEz,'LineWidth',line_width)
plot3(rEx(1),rEy(1),rEz(1),'--o')
xlabel('x [m]','FontSize', label_size, 'Interpreter', 'latex')
ylabel('y [m]','FontSize', label_size, 'Interpreter', 'latex')
zlabel('z [m]','FontSize', label_size, 'Interpreter', 'latex')

%% Geometric path with frames
f = figure();
set(f,'renderer','Painters');
% f.WindowState = "maximized";
% title("Geometric path")
hold on
grid on
box on
axis equal
view(3)
% view([90 0])

% Plot the trajectory
plot3(rEx,rEy,rEz,'LineWidth',line_width)
plot3(rEx(1),rEy(1),rEz(1),'--o')  % Mark the start point

xlabel('x [m]','FontSize', 18, 'FontWeight','bold', 'Interpreter', 'latex')
ylabel('y [m]','FontSize', 18, 'FontWeight','bold','Interpreter', 'latex')
zlabel('z [m]','FontSize', 18, 'FontWeight','bold','Interpreter', 'latex')

% Define frame size
frame_length = 0.1;  % Length of the axis lines

% Define identity frame (XYZ unit vectors)
I = [1; 0; 0];  % X-axis (Red)
J = [0; 1; 0];  % Y-axis (Green)
K = [0; 0; 1];  % Z-axis (Blue)

% START FRAME (No rotation)
start_point = [rEx(1); rEy(1); rEz(1)];
quiver3(start_point(1), start_point(2), start_point(3), ...
        frame_length*I(1), frame_length*I(2), frame_length*I(3), 'r', 'LineWidth', 3) % Red X-axis
quiver3(start_point(1), start_point(2), start_point(3), ...
        frame_length*J(1), frame_length*J(2), frame_length*J(3), 'g', 'LineWidth', 3) % Green Y-axis
quiver3(start_point(1), start_point(2), start_point(3), ...
        frame_length*K(1), frame_length*K(2), frame_length*K(3), 'b', 'LineWidth', 3) % Blue Z-axis


% MIDPOINT FRAME (Rotated by rot/2)
mid_index = round(length(rEx) / 2 +20); % Find middle index
mid_point = [rEx(mid_index); rEy(mid_index); rEz(mid_index)];

rot_z_half = [cos(rot/2), -sin(rot/2), 0;
              sin(rot/2),  cos(rot/2), 0;
              0,           0,          1];  % Rotation matrix for rot/2

I_half = rot_z_half * I;
J_half = rot_z_half * J;
K_half = rot_z_half * K;

quiver3(mid_point(1), mid_point(2), mid_point(3), ...
        frame_length*I_half(1), frame_length*I_half(2), frame_length*I_half(3), 'r', 'LineWidth', 3) % Red X-axis
quiver3(mid_point(1), mid_point(2), mid_point(3), ...
        frame_length*J_half(1), frame_length*J_half(2), frame_length*J_half(3), 'g', 'LineWidth', 3) % Green Y-axis
quiver3(mid_point(1), mid_point(2), mid_point(3), ...
        frame_length*K_half(1), frame_length*K_half(2), frame_length*K_half(3), 'b', 'LineWidth', 3) % Blue Z-axis


% END FRAME (Rotated around Z by 'rot' angle)
end_point = [rEx(end); rEy(end); rEz(end)];
rot_z = [cos(rot), -sin(rot), 0;
         sin(rot),  cos(rot), 0;
         0,        0,        1];  % Rotation matrix around Z

% Apply rotation to identity vectors
I_rot = rot_z * I;
J_rot = rot_z * J;
K_rot = rot_z * K;

% Plot rotated frame at end point
quiver3(end_point(1), end_point(2), end_point(3), ...
        frame_length*I_rot(1), frame_length*I_rot(2), frame_length*I_rot(3), 'r', 'LineWidth', 3) % Red X-axis
quiver3(end_point(1), end_point(2), end_point(3), ...
        frame_length*J_rot(1), frame_length*J_rot(2), frame_length*J_rot(3), 'g', 'LineWidth', 3) % Green Y-axis
quiver3(end_point(1), end_point(2), end_point(3), ...
        frame_length*K_rot(1), frame_length*K_rot(2), frame_length*K_rot(3), 'b', 'LineWidth', 3) % Blue Z-axis


% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
set(gca, 'FontSize', 18, 'TickLabelInterpreter', 'latex');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
% set(gca, 'FontSize',18,'LineWidth',2)
% set(gcf,'PaperPositionMode','auto')

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])

% f.Units = "inches";
% f.Position = [0,0,6,4];

% set(f,'PaperSize',f.Position(3:4));

% Save figure as PDF in 'Frames' folder
if save_fig
    folder_name = 'Data/280225/Figures';
    % print(folder_name,'-dpng','-r0')
    % save_name = 'Data/280225/Figures/ark3_geometric_path.pdf';
    if ~exist(folder_name, 'dir')
        mkdir(folder_name); % Create folder if it doesn't exist
    end
    plot_name = strcat(path_type,'.pdf');
    % set(gcf,'PaperPositionMode','auto')
    print('-dpdf', '-fillpage', plot_name)
    % saveas(f, plot_name);
end


%% Creo Export
Tin     = 1;
n_in    = Tin*freq + 1;
time_in = linspace(0,Tin,n_in);
[q_in,qd_in,qdd_in] = motion_law(zeros(1,6),q_0,zeros(1,6),zeros(1,6),time_in);
time_tot = [time_in, Tin + time(2:end)'];
n_tot = length(time_tot);

q_tot   = zeros(6,n_tot);
qd_tot  = zeros(6,n_tot);
qdd_tot = zeros(6,n_tot);

for i = 1:6
    
    q_tot(i,:)  = [q_in(i,:) q(i,2:end)];% + jointOffset(i);
    qd_tot(i,:) = [qd_in(i,:) qd(i,2:end)];
    name = strcat('comau','_q',num2str(i),'.tab');
    joint = [time_tot' 180/pi*q_tot(i,:)'];
    % save(name,'joint','-ascii','-tabs');
   
end

figure()
hold on
grid on
box on
plot(time_tot,q_tot,'LineWidth',2)
xlabel('t [s]','interpreter','latex','FontSize',14)
ylabel('${q}_{i}$','interpreter','latex','FontSize',14)
legend('1','2','3','4','5','6','interpreter','latex','FontSize',10)

figure()
hold on
grid on
box on
plot(time_tot,qd_tot,'LineWidth',2)
xlabel('t [s]','interpreter','latex','FontSize',14)
ylabel('$\dot{q}_{i}$','interpreter','latex','FontSize',14)
legend('1','2','3','4','5','6','interpreter','latex','FontSize',10)

%% CSV export
pos_x_in = rEx(1)*ones(1,n_in);
pos_y_in = rEy(1)*ones(1,n_in);
pos_z_in = rEz(1)*ones(1,n_in);
if ~isempty(th)
    th_in   = th(1)*ones(1,n_in);
else
    th_in   = zeros(1,n_in);
end

time_tot  = [time_in, Tin + time(2:end)'];
pos_x_tot = [pos_x_in rEx(2:end)] - rEx(1);
pos_y_tot = [pos_y_in rEy(2:end)] - rEy(1);
pos_z_tot = [pos_z_in rEz(2:end)] - rEz(1);
if ~isempty(th)
    th_tot    = [th_in th(2:end)];
else 
    th_tot    = zeros(1,n_tot);
end

q_tot_csv  = [ones(n_in,1)*q_0; q_sol(:,2:end)'];% + jointOffset;

if save_csv

    for i=1:n_tot
        
        q=rotm2quat(Rz(th_tot(i)));
        
        matrix{i,1} = time_tot(i);
        matrix{i,2} = pos_x_tot(i);
        matrix{i,3} = pos_y_tot(i);
        matrix{i,4} = pos_z_tot(i);
        matrix{i,5} = q(2);
        matrix{i,6} = q(3);
        matrix{i,7} = q(4);
        matrix{i,8} = q(1);
    end
    
    save_name=strcat(fig_name,'.csv')
    writecell(matrix,save_name,'Delimiter',';')

    for i=1:n_tot
        
        % joint_matrix{i,1} = time_tot(i);
        joint_matrix{i,1} = q_tot_csv(i,1);
        joint_matrix{i,2} = q_tot_csv(i,2);
        joint_matrix{i,3} = q_tot_csv(i,3);
        joint_matrix{i,4} = q_tot_csv(i,4);
        joint_matrix{i,5} = q_tot_csv(i,5);
        joint_matrix{i,6} = q_tot_csv(i,6);

    end
    
    save_name=strcat(fig_name,'.csv')
    writecell(matrix,save_name,'Delimiter',';')
    save_name=strcat(fig_name,'_joint','.csv')
    writecell(joint_matrix,save_name,'Delimiter',';')

end

figure()
hold on
grid on
box on
plot(time_tot,pos_x_tot)
plot(time_tot,pos_y_tot)
plot(time_tot,pos_z_tot)

figure()
hold on
grid on
box on
plot(time_tot,q_tot_csv')

if ~isempty(th)
    figure()
    hold on
    grid on
    box on
    plot(time,thd)
end







