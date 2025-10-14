%--------------------------------------------------------------------------
% Trajectory Planning for MATRIX project (Comau SmartSix robot) - Tilting
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

ws_path = fullfile('..', '..', '..', '..');
kin_path = fullfile(ws_path,"Kinematics");
odes_path = fullfile(ws_path,"Sloshing_model", "odes");
% add folder and subfolders
addpath(genpath(kin_path));
addpath(genpath(odes_path));

addpath(genpath('utils\'));
% addpath(genpath('ode\'));
addpath(genpath('Data\'));
%%

robot       = create_SmartSix();
jointOffset = [0, 0, pi/2, 0, 0, 0];
toolOffset  = [0 0 0.08]';

path_type   = 'Tilt_y';
dim_type    = '2D';
% motion_type = [path_type,'_',dim_type];
motion_type = path_type;
Te          = 10.2;
Dz_max      = 0.6;

robot_visu = importrobot('BACKUP_comau_smartsix5.urdf.xacro');
robot_visu.DataFormat = 'row'; % Options: 'row', 'column', or 'struct'

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 1;
save_csv   = 1;
save_fig   = 1;

%%
R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, Cs, ms, ks, cs, as, l, J, k, wn] = Parameters(R, h);

wn_pl = 21.31;
Cs_pl = 0.08;
a  = h/R;
mu = 0.001; 
vc = mu/rho;
ml = rho*V;
% hn = 0.5-(2/(csi11*a))*tanh(csi11*0.5*a);
hn = 1/2*h*(1 - 4*R/(csi11*h)*tanh(csi11*h/(2*R)));
% hn = abs(hn);

Ct=4*pi*vc*(a^2*csi11^2)/(wn*h^2);
cost=(a*pi/h)*sqrt(vc/(2*wn));
Cw=cost*(((csi11^2+1)/(csi11^2-1))-2*csi11*a/sinh(2*csi11*a));
Cb=cost*2*csi11/sinh(2*csi11*a);
Ctot1=(Ct+Cw+Cb);
zita=Ctot1/sqrt(4*pi*pi+Ctot1);

freq = 500;
n    = freq*Te + 1;
time = linspace(0,Te,n);
[sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

%% Trajectories

% displacement from frame 7 to cor expressed in frame 0
dx = 0;
dy = -0.2;
% dz = 0;
dz = h/2+hn;

cor = [dx;dy;dz];

[rEx, rEdx, rEddx, rEy, rEdy, rEddy, rEz, rEdz, rEddz, qOffset, psi, psid, psidd, axisDist, A_psi] = TILT_traj(path_type, dim_type, sigma, sigmad, sigmadd, time, n, dx, dy, dz, Te, robot);


% fig_name = strcat('Motions_10_10_2024/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
% fig_name = strcat('Motions_22_01_2025/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
% fig_name = strcat('Motions_22_01_2025/',motion_type,'_', num2str(axisDist),'mz_',num2str(a),'my_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
% fig_name = strcat('Motions_27_01_2025/',motion_type,'_', num2str(dz),'zm_',num2str(dy),'ym_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
fig_name = strcat('Data/310125/Tilt/',motion_type,'_', num2str(dz),'zm_',num2str(dy),'ym_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');


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



% T_off = T(Rz(-pi/4),toolOffset)*T(eye(3),cor);
% T_off = T(Rz(-pi/4),Rz(pi+qOffset(1)-qOffset(6))'*(toolOffset+cor));
% T_off = T(eye(3),toolOffset+cor);

% T_offset = SmartSix_FK(qOffset')*T_off
% T_06_0 = SmartSix_FK(qOffset');

% T_07_0 = SmartSix_FK(qOffset')*T_off



T06_0 = SmartSix_FK(qOffset')

offset = toolOffset+cor; % {p67}0
T67 = T(T06_0(1:3,1:3)', T06_0(1:3,1:3)'*offset); % frame 7 oriented as frame 0
% T67 = T(eye(3), T06_0(1:3,1:3)'*offset);

% T07_0 = SmartSix_FK(qOffset')*T(eye(3),toolOffset+cor) % displacement on z axis
T07_0 = T06_0*T67;
% T78 = T(T07_0(1:3,1:3)', zeros(3,1));
% T08_0 = T07_0*T78;


rE   = [rEx; rEy; rEz];
rEd  = [rEdx; rEdy; rEdz];
rEdd = [rEddx; rEddy; rEddz];

% rE   = [rEx; rEy; rEz];
% rEd  = [rEdx; rEdy_act; rEdz];
% rEdd = [rEddx; rEddy_act; rEddz_act];
% rEddy = rEddy_act;
% rEddz = rEddz_act;

rEy_act = rEy
rEz_act = rEz
rEddy_act = rEddy
rEddz_act = rEddz



% rotation about the base x frame
wE   = [-psid; zeros(1,n); zeros(1,n)];

%% Comau Inverse Kinematics

for i = 1:n
    
    acc_norm(i)   = norm(rEdd(:,i));
    acc_norm2D(i) = norm(rEdd(1:2,i));

    % T06 = [Rx(psi(i)), rE(:,i);
    %        0   0   0     1   ];
    % T06 = [Rx(-psi(i)), T_offset(1:3,1:3)*rE(:,i);
    %        0   0   0     1   ];
    % T7F = [Rx(psi(i)), Rz(pi)'*rE(:,i);
    %        0   0   0     1   ];
    % % T7F = [Rx(psi(i)), T_07_0(1:3,1:3)*rE(:,i);
    % %        0   0   0     1   ];
    % TTT(:,:,i) = T7F;
    % T07_t(:,:,i) = T_07_0*T7F;
    % 
    % T06 = T_07_0*T7F*inv(T_off);
    % TFF(:,:,i) = T06;
    % % q_sol(:,i) = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
    % % q_sol(:,i) = SmartSix_IK(robot,T_offset*T06*inv(T_off),0);
    % q_sol(:,i) = SmartSix_IK(robot,T_07_0*T7F*inv(T_off),0);
    % 
    % 
    T_rot = T(Rx(-psi(i)), zeros(3,1));
    T_disp = T(eye(3), rE(:,i));


    T07 = T_disp*T07_0*T_rot;
    T07_t(:,:,i) = T07;
    T06 = T07*inv(T67);
    T06_t(:,:,i) = T06;
    q_sol(:,i) = SmartSix_IK(robot,T06,0);

    Jg = SmartSix_Jg(q_sol(:,i));
    det(Jg)
    
    R06 = T06(1:3,1:3);
    R07 = T07(1:3,1:3);

    v06 = rEd(:,i) - skew(wE(:,i))*R06*T67(1:3,4);

    qd(:,i) = Jg\[v06; wE(:,i)];


    % % qd(:,i) = Jg\[rEd(:,i)  - skew(wE(:,i))*Rx(psi(i))*toolOffset; wE(:,i)];
    % qd(:,i) = Jg\[rEd(:,i)  - skew(wE(:,i))*T06(1:3,1:3)*T_off(1:3,4); wE(:,i)];

    % qd(:,i) = Jg\[T_offset(1:3,1:3)*rEd(:,i)  - skew(T_offset(1:3,1:3)*wE(:,i))*T_offset(1:3,1:3)*Rx(-psi(i))*toolOffset; T_offset(1:3,1:3)*wE(:,i)];

end
T06 = [Rx(psi(1)), rE(:,1);
       0   0   0     1   ];
% T06 = [Rx(-psi(1)), T_offset(1:3,1:3)*rE(:,1);
%        0   0   0     1   ];
% q_0 = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
% q_0 = SmartSix_IK(robot,T_07_0*T7F*inv(T_off),0);
% q_0 = SmartSix_IK(robot,T_07_0*inv(T_off),0);
q_0 = qOffset;

rad2deg(q_0 + jointOffset)

q = q_0' + cumtrapz(time,qd,2);


figure()
hold on
% plot(time,qOffset' + cumtrapz(time,qd,2))
plot(time, q', "LineStyle","--", "Color","black")
plot(time, q_sol')

%% Animation
animations = 1;
x = zeros(1,n);
y = zeros(1,n);
z = zeros(1,n);
x_dir = zeros(3,n);
y_dir = zeros(3,n);
z_dir = zeros(3,n);
for i=1:n
    x(i)=T07_t(1,4,i);
    y(i)=T07_t(2,4,i);
    z(i)=T07_t(3,4,i);
    x_dir(:,i)=T07_t(1:3,1,i);
    y_dir(:,i)=T07_t(1:3,2,i);
    z_dir(:,i)=T07_t(1:3,3,i);
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
    
    % Initialize quiver handles
    hx = []; 
    hy = []; 
    hz = [];
    % Loop through each configuration in the joint trajectory
    for i = 1:10:numSteps
        % tic
        % Extract the current joint configuration
        currentConfig = jointTrajectory(:, i)';
        
        % Display the robot at the current configuration
        show(robot_visu, currentConfig, 'Frames', 'on', 'PreservePlot', false);
        % Delete previous quivers (if they exist)
        if ~isempty(hx)
            delete(hx);
            delete(hy);
            delete(hz);
        end
        hx = quiver3(x(i), y(i), z(i), x_dir(1,i), x_dir(2,i), x_dir(3,i), 0.07, 'r', 'LineWidth', 1.5);  % x-axis
        hy = quiver3(x(i), y(i), z(i), y_dir(1,i), y_dir(2,i), y_dir(3,i), 0.07, 'g', 'LineWidth', 1.5);  % y-axis
        hz = quiver3(x(i), y(i), z(i), z_dir(1,i), z_dir(2,i), z_dir(3,i), 0.07, 'b', 'LineWidth', 1.5);  % z-axis
        
        % Wait for the next loop to maintain the rate
        waitfor(rateCtrl);
        % toc
    end
end


%% Sloshing-Height Formulation
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0=[0 0 0 0];

% distance from cor and mass H = h/2 + hn, for it to be 0 h/2+hn = 0 => h = -2*hn
h = -2*hn;
% h = 0;

[tNL,sNL]=ode45(@(t,s)odeTiltMSD(t,s,ks,k,zita,ms,time,rEddy,-rEddx,rEddz,psi,psid,psidd,h,hn,g,as,2,'NL'),tspan,S0);

gammaNL = (h*ms*csi11^2)/(rho*V*R);
etaNL = gammaNL*(sNL(:,1).^2 + sNL(:,2).^2).^0.5;

%%Linear Sloshing Model
tspan = [0 2*Te];
S0=[0 0 0 0];

[tL,sL]=ode45(@(t,s)odeTiltMSD(t,s,ks,k,zita,ms,time,rEddy,-rEddx,rEddz,psi,psid,psidd,h,hn,g,as,2,'L'),tspan,S0);

gammaL = (4*h*ms)/(rho*V*R);
etaL = gammaL*(sL(:,1).^2 + sL(:,2).^2).^0.5;

%% Graphics
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;

max_slosh = max(max(etaNL*1000),max(etaL*1000));
Ylim      = (floor(max_slosh/5)+1)*5;


figure()
hold on
grid on 
plot(time,rEy_act)
% plot(time,rEy_act2)



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
plot(tL, etaL*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
plot(tNL, etaNL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
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
plot(time,psi,'LineWidth',line_width)
plot(time,psid,'LineWidth',line_width)
plot(time,psidd,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\psi$ [rad]','$\dot \psi$ [rad/s]', '$\ddot \psi$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,4)
hold on
grid on
box on
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
text(0.5,2.5,['$|\ddot{\psi}|_{max} = $',num2str(max((abs(psidd)))),'[rad/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,2.0,['$\overline \eta_{L,max} = $',num2str(max(etaL*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(0.5,1.5,['$\overline \eta_{NL,max} = $',num2str(max(etaNL*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
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
    
    q_tot(i,:)  = [q_in(i,:) q_sol(i,2:end)];% + jointOffset(i);
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
psi_in   = psi(1)*ones(1,n_in);

time_tot  = [time_in, Tin + time(2:end)];
pos_x_tot = [pos_x_in rEx(2:end)] - rEx(1);
pos_y_tot = [pos_y_in rEy(2:end)] - rEy(1);
pos_z_tot = [pos_z_in rEz(2:end)] - rEz(1);
psi_tot   = [psi_in psi(2:end)];

q_tot_csv  = [ones(n_in,1)*q_0; q_sol(:,2:end)']; %+ jointOffset;

if save_csv

    for i=1:n_tot
        
        q=rotm2quat(Rx(psi_tot(i)));
        
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

    % for i=1:n_tot
    % 
    %     joint_matrix{i,1} = time_tot(i);
    %     joint_matrix{i,2} = q_tot_csv(i,1);
    %     joint_matrix{i,3} = q_tot_csv(i,2);
    %     joint_matrix{i,4} = q_tot_csv(i,3);
    %     joint_matrix{i,5} = q_tot_csv(i,4);
    %     joint_matrix{i,6} = q_tot_csv(i,5);
    %     joint_matrix{i,7} = q_tot_csv(i,6);
    % 
    % end

    for i=1:n_tot
        
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
