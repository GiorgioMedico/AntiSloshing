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

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
% addpath(genpath('..\..\..\Comau_Kinematics\'));
% addpath(genpath('..\utils\'));
% addpath(genpath('utils\'));
% addpath(genpath('ode\'));
% addpath(genpath('Data\'));

%% import 
% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
% addpath(genpath('..\..\..\Comau_Kinematics\'));
% addpath(genpath('..\utils\'));

ws_path = fullfile('..');
% kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
% add folder and subfolders
% addpath(genpath(kin_path));
% addpath(genpath(odes_path));

addpath(genpath('utils\'));
addpath(genpath('odes'));
addpath(genpath('Comau_Kinematics'));
% addpath(genpath('Data\'));

%%
robot       = create_SmartSix();
jointOffset = [0, 0, pi/2, 0, 0, 0];
toolOffset  = [0 0 0.08]';

robot_visu = importrobot('comau_smartsix5.urdf.xacro');
robot_visu.DataFormat = 'row'; % Options: 'row', 'column', or 'struct'

path_type   = 'Line';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
% Te          = 20;
Te = 1.5;
T_InFin     = 3;
Dz_max      = 0.6;
no_Rot      = 0;

save_video = 0;
save_csv   = 0;
save_fig   = 0;

%% Parameters
R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, Cs, ms, ks, cs, as, l, J, k, wn] = Parameters(R, h);

a  = h/R;
% mu = 0.001; 
% vc = mu/rho;
% ml = rho*V;
% hn = 0.5-(2/(csi11*a))*tanh(csi11*0.5*a);
hn = 1/2*h*(1 - 4*R/(csi11*h)*tanh(csi11*h/(2*R)));

freq = 500;
n    = freq*Te + 1;
time = linspace(0,Te,n);
[sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

%% Trajectory
[rEx, rEdx, rEddx, rEy, rEdy, rEddy, rEz, rEdz, rEddz, qOffset, th, thd, thdd, axisDist, th_max] = SCARA_traj(path_type, dim_type, sigma, sigmad, sigmadd, time, n, Te, T_InFin, robot);

Deltath = max(th)-min(th);

if no_Rot
    th   = zeros(1,n);
    thd  = zeros(1,n);
    thdd = zeros(1,n);
    noRot_txt = 'noRot_';
else
    noRot_txt = '';
end

if strcmp(path_type,'FV')
    fig_name = strcat('Motions_13_11_2024/',motion_type,'_',noRot_txt, num2str(axisDist),'m_',num2str(Te),'s_',num2str(Deltath),'rad');
else
    fig_name = strcat('Motions_13_11_2024/',motion_type,'_',noRot_txt, num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
end

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
wE   = [zeros(1,n); zeros(1,n); thd];
wEd   = [zeros(1,n); zeros(1,n); thdd];

%% Comau Inverse Kinematics

for i = 1:n
    
    vel_norm(i)   = norm(rEd(:,i));
    acc_norm(i)   = norm(rEdd(:,i));
    acc_norm2D(i) = norm(rEdd(1:2,i));


    T_rot = T(Rz(th(i)), zeros(3,1));
    T_disp = T(eye(3), rE(:,i));

    T07 = T_disp*T07_0*T_rot;
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


    R07 = T07(1:3,1:3);
    w7_0 = R07*wE(:,i);

    v06 = rEd(:,i) + skew(w7_0)*T67(1:3,4);

    qd(:,i) = Jg\[v06; w7_0];

    % qd(:,i) = Jg\[rEd(:,i) - skew(wE(:,i))*Rz(th(i))*toolOffset; wE(:,i)];

end
T06 = [Rz(th(1)), rE(:,1);
       0   0   0     1   ];
% T06 = [Rz(th(1)), T_offset(1:3,1:3)*rE(:,1);
%        0   0   0     1   ];
% q_0 = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
% rad2deg(q_0 + jointOffset)

q_0 = qOffset;

q = q_0' + cumtrapz(time,qd,2);

max(abs(vel_norm))

q_sol(6,:)=q_sol(6,:)+qOffset(6);
figure()
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
    % view(3);
    % view([2 0 0]);
    view([0 T06_0(1,4) 0]);
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
        
        % Wait for the next loop to maintain the rate
        waitfor(rateCtrl);
        % toc
    end
end

%% Sloshing-Height Formulation
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with paraboloic term 
[tNL,sNL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddx,rEddy,rEddz,thdd_zero,J,g,as,2,'NL'), tspan, S0);
gammaNL = (h*ms*csi11^2)/(rho*V*R);
etaNL = gammaNL*(sNL(:,1).^2 + sNL(:,2).^2).^0.5;

for i = 1:length(tNL)
    if tNL(i)<Te
        thetad(i) = spline(time,thd,tNL(i));
        etaPar(i) = R^2*thetad(i)^2/(4*g);
    else
        etaPar(i) = 0;
    end
end
etaNLPar = etaNL + etaPar';

%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with paraboloic term 
[tL,sL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddx,rEddy,rEddz,thdd_zero,J,g,as,2,'L'), tspan, S0);
gammaL = (4*h*ms)/(rho*V*R);
etaL = gammaL*(sL(:,1).^2 + sL(:,2).^2).^0.5;

clear etaPar
for i = 1:length(tL)
    if tL(i)<Te
        thetad(i) = spline(time,thd,tL(i));
        etaPar(i) = R^2*thetad(i)^2/(4*g);
    else
        etaPar(i) = 0;
    end
end
etaLPar = etaL + etaPar';

%% Graphics
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
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
plot(time,th,'LineWidth',line_width)
plot(time,thd,'LineWidth',line_width)
plot(time,thdd,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\theta$ [rad]','$\dot \theta$ [rad/s]', '$\ddot \theta$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,4)
hold on
grid on
box on
plot(time,q./robot.q_max','LineWidth',line_width)
plot(time,q_sol./robot.q_max','--k')
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

%% Creo Export
Tin     = 1;
n_in    = Tin*freq + 1;
time_in = linspace(0,Tin,n_in);
[q_in,qd_in,qdd_in] = motion_law(zeros(1,6),q_0,zeros(1,6),zeros(1,6),time_in);
time_tot = [time_in, Tin + time(2:end)];
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
th_in   = th(1)*ones(1,n_in);

time_tot  = [time_in, Tin + time(2:end)];
pos_x_tot = [pos_x_in rEx(2:end)] - rEx(1);
pos_y_tot = [pos_y_in rEy(2:end)] - rEy(1);
pos_z_tot = [pos_z_in rEz(2:end)] - rEz(1);
th_tot    = [th_in th(2:end)];

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

figure()
hold on
grid on
box on
plot(time,thd)







