%--------------------------------------------------------------------------
% Cfr between MSD and P first mode TILT
%
%    Author:     Roberto Di Leva
%    Email:      roberto.dileva@unibo.it 
%    Date:       April 2025
%--------------------------------------------------------------------------
clear all
close all
clc

ws_path = fullfile('..', '..','..');
kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
odes_path = fullfile("odes/");


addpath(genpath(kin_path));
addpath(genpath(odes_path));
addpath("utils")

path_type   = 'Tilt_TRD';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
Te          = 2.5;
A_psi       = deg2rad(30);
Dz_max      = 0.6;
Delta_start_index = 0;

sloshing_masses = [1 2 3];
R = 0.049;
h = 0.08;
% [g, rho, m_tot, V, csi11, zitan, mn, kn, cs, alphan, l, J, k, wn] = Parameters(R, h);
[g, rho, m_tot, V, csi11, zita1, m1, k1, c1, alphan, l1, L1, J, k, w1] = nModeParameters(R, h, 1);
[g, rho, m_tot, V, csi12, zita2, m2, k2, c2, alphan, l2, L2, J, k, w2] = nModeParameters(R, h, 2);
[g, rho, m_tot, V, csi13, zita3, m3, k3, c3, alphan, l3, L3, J, k, w3] = nModeParameters(R, h, 3);

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 0;
save_fig   = 0;

w = 2;
a  = h/R;
% h1 = h*(0.5-(2/(csi11*a))*tanh(csi11*0.5*a));
% h2 = h*(0.5-(2/(csi12*a))*tanh(csi12*0.5*a));
% h3 = h*(0.5-(2/(csi13*a))*tanh(csi13*0.5*a));

h1 = h/2 - R/csi11*tanh(csi11*h/R);
h2 = h/2 - R/csi12*tanh(csi12*h/R);
h3 = h/2 - R/csi13*tanh(csi13*h/R);

EOMtype = 'NL';

freq = 500;
n    = freq*Te + 1;
time = linspace(0,Te,n);
[sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

if strcmp(path_type,'Tilt_LE')

    axisDist = 0.0; 
    nb = 2;
    a = 0.5;
    b = 1;
    % A_psi = deg2rad(20);
    th_max = nb*pi;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a/2*sin(2*th+pi);
    rEdx  = a*thd.*cos(2*th+pi);
    rEddx = a*thdd.*cos(2*th+pi) - 2*a*thd.^2.*sin(2*th+pi);
    rEy   = a*cos(th+pi/2);
    rEdy  = - a*thd.*sin(th+pi/2);
    rEddy = - a*thdd.*sin(th+pi/2) - a*thd.^2.*cos(th+pi/2);

    psi   = A_psi*sin(2*th);
    psid  = 2*A_psi*thd.*cos(2*th);
    psidd = 2*A_psi*thdd.*cos(2*th) - 4*A_psi*thd.^2.*sin(2*th);
    
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

elseif strcmp(path_type,'Tilt_RD')

    axisDist = 0.3; 
    nb = 4;
    a = axisDist;
    b = 1.5;
    % A_psi = deg2rad(20);
    th_max = nb*pi;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a*sin(th);
    rEdx  = a*thd.*cos(th);
    rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
    rEy   = a*cos(th) - a;
    rEdy  = - a*thd.*sin(th);
    rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);

    psi   = A_psi*sin(th);
    psid  = A_psi*thd.*cos(th);
    psidd = A_psi*thdd.*cos(th) - A_psi*thd.^2.*sin(th);

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
    % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
    % qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
    % qOffset = [0.0000   -0.0994   -1.8237    0.0000   -1.7243    3.1400];
    qOffset = [0.0000   -0.0790   -1.6522    0.0000   -1.5732    3.1400];

elseif strcmp(path_type,'Tilt_TRD')

    axisDist = 0.3; 
    nb = 1.5;
    a = axisDist;
    b = 1;
    % A_psi = deg2rad(30);
    th_max = nb*pi;

    [th,thd,thdd] = motion_law(0,th_max,0,0,time);

    rEx   = a*sin(th);
    rEdx  = a*thd.*cos(th);
    rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
    rEy   = a*cos(th) + b*sigma - a;
    rEdy  = - a*thd.*sin(th) + b*sigmad;
    rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;

    psi   = A_psi*sin(4/3*th);
    psid  = 4/3*A_psi*thd.*cos(4/3*th);
    psidd = 4/3*A_psi*thdd.*cos(4/3*th) - 16/9*A_psi*thd.^2.*sin(4/3*th);

    if strcmp(dim_type,'3D')
        rEz   = (a/2)*sin(4/3*th);
        rEdz  = (2*a/3)*thd.*cos(4/3*th);
        rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
    elseif strcmp(dim_type,'2D')
        rEz   = zeros(1,n);
        rEdz  = zeros(1,n);
        rEddz = zeros(1,n);
    else
        fprintf('Wrong dimension type...')
    end    
    % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
    % qOffset = [0.6,0.0,-1.8,0.0,-1.8,3.14];
    qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
    
end
fig_name = strcat('Motions_10_10_2024/sloshMasses/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
post_name = strcat('Motions_10_10_2024/full_slosh/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg','.mat');
type_name = strcat(motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');

rE   = [rEx; rEy; rEz];
rEd  = [rEdx; rEdy; rEdz];
rEdd = [rEddx; rEddy; rEddz];
wE   = [zeros(1,n); zeros(1,n); thd];
load(post_name);

if strcmp(path_type,'Tilt_LE') 
    psiOde = - psi;
    psidOde = - psid;
    psiddOde = - psidd;
elseif strcmp(path_type,'Tilt_RD')
    psiOde = - psi;
    psidOde = - psid;
    psiddOde = - psidd;
elseif strcmp(path_type,'Tilt_TRD')
    psiOde = psi;
    psidOde = psid;
    psiddOde = psidd;
end

%% Comau Inverse Kinematics

for i = 1:n
    
    vel_norm(i)   = norm(rEd(:,i));
    acc_norm(i)   = norm(rEdd(:,i));
    acc_norm2D(i) = norm(rEdd(1:2,i));

end

%% Sloshing-Height Computation with Varying sloshing masses
tspan = [0 2*Te];
S0 = [0 0 0 0];

gammaNL1 = (h*m1*csi11^2)/(rho*V*R);
gammaNL2 = (h*m2*csi12^2)/(rho*V*R);
gammaNL3 = (h*m3*csi13^2)/(rho*V*R);

gammaPNL1 = (h*m1*csi11)/(rho*V*tanh(csi11*h/R));
gammaPNL2 = (h*m2*csi12)/(rho*V*tanh(csi12*h/R));
gammaPNL3 = (h*m3*csi13)/(rho*V*tanh(csi13*h/R)); 

time_spline = linspace(0,2*Te,2*Te*500+1);
psi_spline = linspace(0,2*pi,361);
r_spline = linspace(0,R,50);

[tNLt1,sNLt1] = ode45(@(t,s)odeTiltMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h1,g,alphan,w,EOMtype), tspan, S0);
etaNLt1 = gammaNL1*(sNLt1(:,1).^2 + sNLt1(:,2).^2).^0.5;
phiNLt1 = atan2(sNLt1(:,2),sNLt1(:,1));
xnNLt1 = sNLt1(:,1);
ynNLt1 = sNLt1(:,2);

[tPNLt1,sPNLt1] = ode45(@(t,s)odeTiltP(t,s,l1,L1,zita1,m1,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h1,g,alphan,w1,EOMtype),tspan,S0);
phixPNLt1 = sPNLt1(:,2);
phiyPNLt1 = sPNLt1(:,1);
xnPNLt1 = l1*sin(phiyPNLt1).*cos(phixPNLt1);
ynPNLt1 = l1*sin(phixPNLt1);
phiPNLt1 = atan2(ynPNLt1,xnPNLt1);

etaPNLt1radial = gammaPNL1*sqrt(sin(phiyPNLt1).^2.*cos(phixPNLt1).^2 + sin(phixPNLt1).^2);
etaPNLt1vertical = gammaPNL1*sqrt(2*(1 - cos(phixPNLt1).*cos(phiyPNLt1)));
etaPNLt1MSD = gammaNL1*(xnPNLt1.^2 + ynPNLt1.^2).^0.5;

%% Phasing procedure
%--------------------------------------------------------------------------
%%Linear Sloshing Model
gammaL = (4*h*m1)/(rho*V*R);
[tLt,sLt] = ode45(@(t,s)odeTiltMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h1,g,alphan,w,'L'), tspan, S0);
etaLt = gammaL*(sLt(:,1).^2 + sLt(:,2).^2).^0.5;
%--------------------------------------------------------------------------

Efps = 60; % Framerate experiment video
end_time = 1.5*Te;
[~, emax_index] = max(slosh_heights);
[~, Mmax_index] = max(etaLt);
peak_time = tLt(Mmax_index);
E_start_index = round(emax_index - peak_time*Efps) + Delta_start_index;
E_slosh_h_cut_1 = slosh_heights(E_start_index+1:E_start_index+(end_time*Efps));
E_slosh_t_cut_1 = slosh_times(E_start_index+1:E_start_index+(end_time*Efps)) - slosh_times(E_start_index+1);

%% 

% Create video writer object
v = VideoWriter('Cfr_tilt_video.mp4', 'MPEG-4');
v.FrameRate = 30;  % adjust as needed
open(v)

% 
figure()
set(gcf, 'Position', get(0, 'Screensize'));
hold on
grid on
axis equal
xlim(1000*[-R-0.01 R+0.01])
ylim(1000*[-R-0.01 R+0.01])
zlim(1000*[0 0.15])
view([0 0])   %xz
% view([90 0])  %zy
% view([90 90]) %xy
% view(3)
xlabel('x [m]','interpreter','latex','FontSize',14)
ylabel('y [m]','interpreter','latex','FontSize',14)
zlabel('z [m]','interpreter','latex','FontSize',14)
[xrec,yrec,zrec] = cylinder(1000*(R+0.005),19);
zrec = 150*zrec;
% [x,y,z] = paraboloid(20);
[xpar,ypar] = meshgrid(-1000*R:1000*R,-1000*R:1000*R);
zpar = w1^2/2/1000/g*(xpar.^2 + ypar.^2);
[x,y,z] = sphere(20);
xsph = 1000*l1*x; ysph = 1000*l1*y; zsph = 1000*l1*z + 1000*h - 1000*L1;

recipient = surf(xrec,yrec,zrec,'FaceColor','w','FaceAlpha',0.2,'EdgeColor','k','LineStyle',':');
paraboloid = surf(xpar,ypar,1000*(h/2+h1) + zpar,'FaceColor','b','FaceAlpha',0.05,'EdgeColor','b','LineStyle',':');
pend_sphere = surf(xsph,ysph,zsph,'FaceColor','r','FaceAlpha',0.05,'EdgeColor','r','LineStyle',':');
for i = 1:length(tNLt1)

    pend_rod = plot3([0, 1000*xnPNLt1(i)],[0, 1000*ynPNLt1(i)],[1000*h - 1000*L1,-1000*l1*cos(phiyPNLt1(i))*cos(phixPNLt1(i)) + 1000*h - 1000*L1],'Color','k');
    pend_mass = plot3(1000*xnPNLt1(i),1000*ynPNLt1(i),-1000*l1*cos(phiyPNLt1(i))*cos(phixPNLt1(i)) + 1000*h - 1000*L1,'-o','Color','r','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
    MSD_mass = plot3(1000*xnNLt1(i),1000*ynNLt1(i),1000*(h/2 + h1 + w1^2/2/g*(xnNLt1(i)^2 + ynNLt1(i)^2)),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
    % pause(0.1)

    % Capture frame for video
    frame = getframe(gcf);
    writeVideo(v, frame);

    delete(pend_rod)
    delete(pend_mass)
    delete(MSD_mass)

end

% Finalize video
close(v)

%%
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;
Ylim = 80;

max_slosh_mod = max(etaLt*1000);
max_slosh = max(max_slosh_mod,max(E_slosh_h_cut_1));
Ylim      = (floor(max_slosh/5)+1)*5 + 10;

fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle(fig_name,'Interpreter', 'latex')
subplot(2,3,1)
hold on
grid on
box on
xlim([0 1.5*Te])
plot(tNLt1,1000*xnNLt1,'LineWidth',0.6*line_width)
plot(tPNLt1,1000*xnPNLt1,'LineWidth',0.6*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_1$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,2)
hold on
grid on
box on
xlim([0 1.5*Te])
plot(tNLt1,1000*ynNLt1,'LineWidth',0.6*line_width)
plot(tPNLt1,1000*ynPNLt1,'LineWidth',0.6*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$y_1$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,3)
hold on
grid on
box on
xlim([0 1.5*Te])
plot(tNLt1,1000*sqrt(xnNLt1.^2 + ynNLt1.^2),'LineWidth',0.6*line_width)
plot(tPNLt1,1000*sqrt(xnPNLt1.^2 + ynPNLt1.^2),'LineWidth',0.6*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\sqrt{x_1^2 + y_1^2}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on
xlim([0 1.5*Te])
plot(tNLt1,1000*(h1 + w1^2/2/g*(xnNLt1.^2 + ynNLt1.^2)),'LineWidth',0.6*line_width)
plot(tPNLt1,1000*(-l1*cos(phiyPNLt1).*cos(phixPNLt1) + h/2 - L1),'LineWidth',0.6*line_width)
% plot(tNLt1,(h - L1 - sqrt(l1^2 - xnNLt1.^2 - ynNLt1.^2)),'--k')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$z_1$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim])
plot(tNLt1,1000*etaNLt1,'LineWidth',0.6*line_width)
plot(tPNLt1,1000*etaPNLt1radial,'LineWidth',0.6*line_width)
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),':','LineWidth',0.6*line_width,'Color','#7E2F8E')
% plot(tPNLt1,1000*etaPNLt1MSD,'--k')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline{\eta}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P - radial','Experiment','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'-Bessel - n=1}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,6)
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim])
plot(tNLt1,1000*etaNLt1,'LineWidth',0.6*line_width)
plot(tPNLt1,1000*etaPNLt1vertical,'LineWidth',0.6*line_width)
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),':','LineWidth',0.6*line_width,'Color','#7E2F8E')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline{\eta}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('MSD','P - vertical','Experiment','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'-Bessel - n=1}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
if save_fig
    save_name = strcat(fig_name,'_',EOMtype,'_CfrMSDvsP','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    % save_name = strcat(fig_name,'_sloshMasses_plot1','.pdf');
    % print('-dpdf', '-fillpage', save_name)
end

%%

fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
subplot(2,3,1)
hold on
grid on
box on 
plot(time,rEddy,'LineWidth',line_width)
plot(time,-rEddx,'LineWidth',line_width)
plot(time,rEddz,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('[m/s$^2$]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\ddot x_0$','$\ddot y_0$', '$\ddot z_0$', 'Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,2)
hold on
grid on
box on 
plot(time,psiOde,'LineWidth',line_width)
plot(time,psidOde,'LineWidth',line_width)
plot(time,psiddOde,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\psi_0$ [rad]','$\dot \psi_0$ [rad/s]', '$\ddot \psi_0$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
box off
axis off
xlim([0 5])
ylim([0 5])
label_size = 11;
text(-1,5,['$||\ddot{\mathbf{S}}_0||_{max} = $',num2str(max(acc_norm)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4.5,['$||\ddot{\mathbf{S}}_{0,2D}||_{max} = $',num2str(max(acc_norm2D)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4,['$|\ddot{z}_0|_{max} = $',num2str(max((abs(rEddz)))),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.5,['$|\ddot{\psi}|_{max} = $',num2str(max((abs(psidd)))),'[rad/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.0,['$\overline \eta_{MSD,max} = $',num2str(max(1000*etaNLt1)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.5,['$\overline \eta_{P-radial,max} = $',num2str(max(1000*etaPNLt1radial)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.0,['$\overline \eta_{P-vertical,max} = $',num2str(max(1000*etaPNLt1vertical)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,1.5,['$\overline \eta_{max} = $',num2str(max(abs(E_slosh_h_cut_1))),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,1.0,['$T_{e} = $',num2str(Te),'[s]'],'FontSize', label_size, 'Interpreter', 'latex')
if save_fig
    save_name = strcat(fig_name,'_motion','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    % save_name = strcat(fig_name,'_sloshMasses_plot1','.pdf');
    % print('-dpdf', '-fillpage', save_name)
end