%--------------------------------------------------------------------------
% Trajectory Planning for MATRIX project (Comau SmartSix robot) - Schoen
%
%    Author:     Roberto Di Leva
%    Email:      roberto.dileva@unibo.it 
%    Date:       September 2024
%--------------------------------------------------------------------------
clear all
close all
clc

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
addpath('Comau_Kinematics');

robot       = create_SmartSix();
jointOffset = [0, 0, pi/2, 0, 0, 0];
toolOffset  = [0 0 0.08]';

path_type   = 'RD';
dim_type    = '3D';
motion_type = [path_type,'_',dim_type];
Te          = 6.3;
T_InFin     = 3;
Dz_max      = 0.6;
no_Rot      = 0;
motion_type_new = 'Sc';

R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, Cs, ms, ks, cs, as, l, J, k, wn] = Parameters(R, h);

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 1;
save_fig   = 0;

wn_pl = 21.31;
Cs_pl = 0.08;
if preon_flag == 1
    Cs = Cs_pl;
    cs = 2*Cs*wn_pl*ms;
    ks = ms*wn_pl^2;
end

freq = 500;
n    = freq*Te + 1;
time = linspace(0,Te,n);
[sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

if strcmp(path_type,'LE')

    axisDist = 0.0; 
    nb = 5;
    a = 0.5;
    b = 1;
    th_max = nb*pi;

    phi   = 2*pi*sigma;
    phid  = 2*pi*sigmad;
    phidd = 2*pi*sigmadd;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a/2*sin(2*phi+pi);
    rEdx  = a*phid.*cos(2*phi+pi);
    rEddx = a*phidd.*cos(2*phi+pi) - 2*a*phid.^2.*sin(2*phi+pi);
    rEy   = a*cos(phi+pi/2);
    rEdy  = - a*phid.*sin(phi+pi/2);
    rEddy = - a*phidd.*sin(phi+pi/2) - a*phid.^2.*cos(phi+pi/2);
    
    if strcmp(dim_type,'3D')
        rEz   = (rEy.^2 - rEx.^2)/b^2;
        rEdz  = (2*rEy.*rEdy - 2*rEx.*rEdx)/b^2;
        rEddz = (2*rEdy.^2 + 2*rEy.*rEddy  - 2*rEdx.^2 - 2*rEx.*rEddx)/b^2;
    elseif strcmp(dim_type,'2D')
        rEz   = zeros(1,n);
        rEdz  = zeros(1,n);
        rEddz = zeros(1,n);
    else
        fprintf('Wrong dimension type...')
    end    
    qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];

elseif strcmp(path_type,'RD')

    axisDist = 0.3; 
    nb = 4;
    a = axisDist;
    b = 1.5;
    th_max = nb*pi;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a*sin(th);
    rEdx  = a*thd.*cos(th);
    rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
    rEy   = a*cos(th) - a;
    rEdy  = - a*thd.*sin(th);
    rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);

    if strcmp(dim_type,'3D')
        rEz   = - (rEx.^2 - (rEy + a).^2)/b^2 - a^2/b^2;
        rEdz  = - (2*rEx.*rEdx - 2*(rEy + a).*rEdy)/b^2;
        rEddz = - (2*rEdx.^2 + 2*rEx.*rEddx  - 2*rEdy.^2 - 2*(rEy + a).*rEddy)/b^2;
    elseif strcmp(dim_type,'2D')
        rEz   = zeros(1,n);
        rEdz  = zeros(1,n);
        rEddz = zeros(1,n);
    else
        fprintf('Wrong dimension type...')
    end    
    qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
    % qOffset = [0.4,0.0,-1.9,0.0,-1.8,-0.4+pi];

elseif strcmp(path_type,'FV')

    axisDist = 0.3; 
    nb = 2;
    a = axisDist;
    b = 1.5;
    th_max = nb*pi;

    % T_InFin = 4;
    T_Cost = Te - T_InFin;
    freq = 500;
    n_Infin = freq*T_InFin + 1;
    n_Cost  = freq*T_Cost +1;
    time_InFin = linspace(0,T_InFin,n_Infin);
    time_Cost  = linspace(0,T_Cost,n_Cost);

    [th_InFin,thd_InFin,thdd_InFin] = motion_law(0,th_max,0,0,time_InFin);

    thdd = [thdd_InFin(1:(n_Infin-1)/2+1), zeros(1,n_Cost-1), thdd_InFin((n_Infin-1)/2+2:end)];
    thd = cumtrapz(time,thdd);
    th = cumtrapz(time,thd);

    rEx   = a*sin(th);
    rEdx  = a*thd.*cos(th);
    rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
    rEy   = a*cos(th) - a;
    rEdy  = - a*thd.*sin(th);
    rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);

    if strcmp(dim_type,'3D')
        rEz   = - (rEx.^2 - (rEy + a).^2)/b^2 - a^2/b^2;
        rEdz  = - (2*rEx.*rEdx - 2*(rEy + a).*rEdy)/b^2;
        rEddz = - (2*rEdx.^2 + 2*rEx.*rEddx  - 2*rEdy.^2 - 2*(rEy + a).*rEddy)/b^2;
    elseif strcmp(dim_type,'2D')
        rEz   = zeros(1,n);
        rEdz  = zeros(1,n);
        rEddz = zeros(1,n);
    else
        fprintf('Wrong dimension type...')
    end    
    qOffset = [0.0,-0.1,-1.9,0.0,-1.8,robot.q_min(6)+pi];
    % qOffset = [0.4,0.0,-1.9,0.0,-1.8,-0.4+pi];

elseif strcmp(path_type,'TRD')

    axisDist = 0.3; 
    nb = 1.5;
    a = axisDist;
    b = 1;
    th_max = nb*pi;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a*sin(th);
    rEdx  = a*thd.*cos(th);
    rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
    rEy   = a*cos(th) + b*sigma - a;
    rEdy  = - a*thd.*sin(th) + b*sigmad;
    rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;

    if strcmp(dim_type,'3D')
        rEz   = (a/2)*sin(4/3*th);
        rEdz  = (2*a/3)*thd.*cos(4/3*th);
        rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
        qOffset = [0.4,0.0,-1.72,0.0,-1.72,-0.4];
    elseif strcmp(dim_type,'2D')
        rEz   = zeros(1,n);
        rEdz  = zeros(1,n);
        rEddz = zeros(1,n);
        qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
    else
        fprintf('Wrong dimension type...')
    end    
    % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
    
    
end
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

if ~strcmpi(motion_type_new,'')
    fig_name = fullfile('csv', 'Traj', motion_type_new);
end

thd_max = max(thd)

T_offset = SmartSix_FK(qOffset')*T(eye(3),toolOffset)
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

    T06 = [Rz(th(i)), rE(:,i);
           0   0   0     1   ];
    % T06 = [Rz(th(i)), T_offset(1:3,1:3)*rE(:,i);
    %        0   0   0     1   ];

    q_sol(:,i) = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
    Jg = SmartSix_Jg(q_sol(:,i));

    qd(:,i) = Jg\[rEd(:,i) - skew(wE(:,i))*Rz(th(i))*toolOffset; wE(:,i)];

end
T06 = [Rz(th(1)), rE(:,1);
       0   0   0     1   ];
% T06 = [Rz(th(1)), T_offset(1:3,1:3)*rE(:,1);
%        0   0   0     1   ];
q_0 = SmartSix_IK(robot,T_offset*T06*inv(T(eye(3),toolOffset)),0);
rad2deg(q_0 + jointOffset)

q = q_0' + cumtrapz(time,qd,2);

max(abs(vel_norm))

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
    
    q_tot(i,:)  = [q_in(i,:) q(i,2:end)] + jointOffset(i);
    qd_tot(i,:) = [qd_in(i,:) qd(i,2:end)];
    name = strcat('comau','_q',num2str(i),'.tab');
    joint = [time_tot' 180/pi*q_tot(i,:)'];
    save(name,'joint','-ascii','-tabs');
   
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
vel_x_in = rEdx(1)*ones(1,n_in);
vel_y_in = rEdy(1)*ones(1,n_in);
vel_z_in = rEdz(1)*ones(1,n_in);
acc_x_in = rEddx(1)*ones(1,n_in);
acc_y_in = rEddy(1)*ones(1,n_in);
acc_z_in = rEddz(1)*ones(1,n_in);
th_in   = th(1)*ones(1,n_in);
thd_in   = thd(1)*ones(1,n_in);
thdd_in   = thdd(1)*ones(1,n_in);

time_tot  = [time_in, Tin + time(2:end)];
pos_x_tot = [pos_x_in rEx(2:end)] - rEx(1);
pos_y_tot = [pos_y_in rEy(2:end)] - rEy(1);
pos_z_tot = [pos_z_in rEz(2:end)] - rEz(1);
vel_x_tot = [vel_x_in rEdx(2:end)];
vel_y_tot = [vel_y_in rEdy(2:end)];
vel_z_tot = [vel_z_in rEdz(2:end)];
acc_x_tot = [acc_x_in rEddx(2:end)];
acc_y_tot = [acc_y_in rEddy(2:end)];
acc_z_tot = [acc_z_in rEddz(2:end)];
th_tot    = [th_in th(2:end)];
thd_tot    = [thd_in thd(2:end)];
thdd_tot    = [thdd_in thdd(2:end)];

w_tot = [zeros(1,length(thd_tot)); zeros(1,length(thd_tot)); thd_tot];
wd_tot = [zeros(1,length(thdd_tot)); zeros(1,length(thdd_tot)); thdd_tot];


q_tot_csv  = [ones(n_in,1)*q_0; q_sol(:,2:end)'] + jointOffset;

if save_csv

    matrix = cell(n_tot, 20);  % Preallocate cell array

    for i = 1:n_tot
        q = rotm2quat(Rz(th_tot(i)));  % Convert rotation to quaternion

        matrix{i,1} = time_tot(i);
        matrix{i,2} = pos_y_tot(i);
        matrix{i,3} = -pos_x_tot(i);
        matrix{i,4} = pos_z_tot(i);
        matrix{i,5} = vel_y_tot(i);
        matrix{i,6} = -vel_x_tot(i);
        matrix{i,7} = vel_z_tot(i);
        matrix{i,8} = acc_y_tot(i);
        matrix{i,9} = -acc_x_tot(i);
        matrix{i,10} = acc_z_tot(i);
        matrix{i,11} = q(2);  % x
        matrix{i,12} = q(3);  % y
        matrix{i,13} = q(4);  % z
        matrix{i,14} = q(1);  % w
        matrix{i,15} = w_tot(1,i); 
        matrix{i,16} = w_tot(2,i); 
        matrix{i,17} = w_tot(3,i); 
        matrix{i,18} = wd_tot(1,i); 
        matrix{i,19} = wd_tot(2,i); 
        matrix{i,20} = wd_tot(3,i); 
    end

    % Convert to table with headers
    T = cell2table(matrix, 'VariableNames', { ...
        'time', 'pos_x', 'pos_y', 'pos_z', ...
        'vel_x', 'vel_y', 'vel_z', ...
        'acc_x', 'acc_y', 'acc_z', ...
        'quat_x', 'quat_y', 'quat_z', 'quat_w', ...
        'ang_vel_x', 'ang_vel_y', 'ang_vel_z',...
        'ang_acc_x', 'ang_acc_y', 'ang_acc_z'});

    % Write to CSV
    writetable(T, strcat(fig_name, '.csv'),'Delimiter', ';');

end

% if save_csv
% 
%     for i=1:n_tot
% 
%         q=rotm2quat(Rz(th_tot(i)));
% 
%         matrix{i,1} = time_tot(i);
%         matrix{i,2} = pos_x_tot(i);
%         matrix{i,3} = pos_y_tot(i);
%         matrix{i,4} = pos_z_tot(i);
%         matrix{i,5} = q(2);
%         matrix{i,6} = q(3);
%         matrix{i,7} = q(4);
%         matrix{i,8} = q(1);
% 
%     end
% 
%     save_name=strcat(fig_name,'.csv')
%     writecell(matrix,save_name,'Delimiter',';')
% 
%     for i=1:n_tot
% 
%         joint_matrix{i,1} = time_tot(i);
%         joint_matrix{i,2} = q_tot_csv(i,1);
%         joint_matrix{i,3} = q_tot_csv(i,2);
%         joint_matrix{i,4} = q_tot_csv(i,3);
%         joint_matrix{i,5} = q_tot_csv(i,4);
%         joint_matrix{i,6} = q_tot_csv(i,5);
%         joint_matrix{i,7} = q_tot_csv(i,6);
% 
%     end
% 
%     save_name=strcat(fig_name,'.csv')
%     writecell(matrix,save_name,'Delimiter',';')
%     save_name=strcat(fig_name,'_joint','.csv')
%     writecell(joint_matrix,save_name,'Delimiter',';')
% 
% end

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






