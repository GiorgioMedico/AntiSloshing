%--------------------------------------------------------------------------
% Sensistivity Analysis for Sloshing-model Validation (MATRIX) - MSD
%
%    Author:     Roberto Di Leva
%    Email:      roberto.dileva@unibo.it 
%    Date:       January 2025
%--------------------------------------------------------------------------
clear all
% close all
clc

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
addpath('Comau_Kinematics');
% addpath('Pendulum');

path_type   = 'TRD';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
Te          = 2.2;
Dz_max      = 0.6;
Delta_start_index = -5;

thddMag = 1;

R = 1.0*0.049;
% R = 2.5*0.049;
h = 0.08;
% h = 3*0.08;
% [g, rho, m_tot, V, csi11, zitan, mn, kn, cs, alphan, l, J, k, wn] = Parameters(R, h);
[g, rho, m_tot, V, csi11, zitan, mn, kn, cn, alphan, ln, Ln, J, k, wn] = ParametersP(R, h);

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 0;
save_fig   = 0;

esp_k_min = -6;
esp_k_max = -1;
% esp_k_min = -4;
% esp_k_max = 1;

k_range = logspace(esp_k_min, esp_k_max, esp_k_max - esp_k_min + 1);
% k_range = logspace(esp_k_min, esp_k_max, 10*(esp_k_max - esp_k_min) + 1);
% alpha_range = [0.42, 0.50, 0.58, 0.66, 0.74];
% alpha_range = 0.4+[0.42, 0.50, 0.58, 0.66, 0.74];
% alpha_range = [0.58, linspace(1/100,10,11)];
w = 2;
alpha_range = [0.58, linspace(0,1,11)]/(R^(2*w-2));
% alpha_range = [0.58, 0, 0.58]/(R^(2*w-2));
% alpha_range = [0.58, linspace(0,10,11)];


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
fig_name = strcat('Motions_10_10_2024/Sens_Analysis/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
post_name = strcat('Motions_10_10_2024/full_slosh/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg','.mat');
type_name = strcat(motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
rE   = [rEx; rEy; rEz];
rEd  = [rEdx; rEdy; rEdz];
rEdd = [rEddx; rEddy; rEddz];
wE   = [zeros(1,n); zeros(1,n); thd];

load(post_name);

%% Comau Inverse Kinematics

for i = 1:n
    
    vel_norm(i)   = norm(rEd(:,i));
    acc_norm(i)   = norm(rEdd(:,i));
    acc_norm2D(i) = norm(rEdd(1:2,i));

end

%% (vd) Sloshing-Height Computation with Varying k and alpha_n
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];

gammaL = (4*h*mn)/(rho*V*R);
gammaNL = (h*mn*csi11^2)/(rho*V*R);

for i = 1:length(k_range)
    
    % Lstr(i).k = k_range(i);
    % [tLk,sLk] = ode45(@(t,s)odeSchoenMSD(t,s,kn,k_range(i),zitan,mn,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan,2,'L'), tspan, S0);
    % etaLk = gammaL*(sLk(:,1).^2 + sLk(:,2).^2).^0.5;
    % 
    % Lstr(i).tLk = tLk;
    % Lstr(i).sLk = sLk;
    % Lstr(i).etaLk = etaLk;

    for j = 1:length(alpha_range)
        
        NLstr(j).alpha(i).alpha = alpha_range(j);
        NLstr(j).alpha(i).k = k_range(i);
        [tNLk,sNLk] = ode45(@(t,s)odeSchoenMSD(t,s,kn,k_range(i),zitan,mn,time,rEddy,-rEddx,rEddz,thddMag*thdd,J,g,alpha_range(j),w,'NL'), tspan, S0);
        etaNLk = gammaNL*(sNLk(:,1).^2 + sNLk(:,2).^2).^0.5;

        NLstr(j).alpha(i).tNLk = tNLk;
        NLstr(j).alpha(i).sNLk = sNLk;
        NLstr(j).alpha(i).etaNLk = etaNLk;

    end

end


%% Sloshing-Height Formulation (3D-NL)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with paraboloic term 
[tNLp,sNL] = ode45(@(t,s)odeSchoenMSD(t,s,kn,k,zitan,mn,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,alphan,2,'NL'), tspan, S0);
gammaNL = (h*mn*csi11^2)/(rho*V*R);
etaNL = gammaNL*(sNL(:,1).^2 + sNL(:,2).^2).^0.5;

%% Figures: Influence of k
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;

fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x'],'Interpreter', 'latex')
subplot(2,3,1)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,1000*(NLstr(1).alpha(j).sNLk(:,1)),'LineWidth',0.6*line_width)
end
plot(tNLp,1000*sNL(:,1),'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_n$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,2)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,1000*(NLstr(1).alpha(j).sNLk(:,2)),'LineWidth',0.6*line_width)
end
plot(tNLp,1000*sNL(:,2),'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$y_n$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,((1000*NLstr(1).alpha(j).sNLk(:,1)).^2),'LineWidth',0.6*line_width)
end
plot(tNLp,(1000*sNL(:,1)).^2,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_n^2$ [mm$^2$]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,((1000*NLstr(1).alpha(j).sNLk(:,2)).^2),'LineWidth',0.6*line_width)
end
plot(tNLp,(1000*sNL(:,2)).^2,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$y_n^2$ [mm$^2$]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,6)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,((1000*NLstr(1).alpha(j).sNLk(:,1)).^2)+((1000*NLstr(1).alpha(j).sNLk(:,2)).^2),'LineWidth',0.6*line_width)
end
plot(tNLp,(1000*sNL(:,1)).^2+(1000*sNL(:,2)).^2,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_n^2 + y_n^2$ [mm$^2$]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,['$\mathrm{NL_{vd}} - \alpha_n = $', num2str(alpha_range(1))],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,3)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,(atan2(NLstr(1).alpha(j).sNLk(:,2),NLstr(1).alpha(j).sNLk(:,1))),'LineWidth',0.6*line_width)
end
plot(tNLp,atan2(sNL(:,2),sNL(:,1)),'--k')
line([0 2*Te],[-pi -pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
line([0 2*Te],[ pi  pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,['$\mathrm{NL_{vd}} - \alpha_n = $', num2str(alpha_range(1))],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_xnyn_MSD','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_xnyn_MSD','.pdf');
    print('-dpdf', '-fillpage', save_name)
end



%%

fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x'],'Interpreter', 'latex')
subplot(2,3,1)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,(NLstr(1).alpha(j).sNLk(:,6)),'LineWidth',0.6*line_width)
end
plot(time,thddMag*thd,'--k')
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\dot \theta_l$ [rad/s]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,2)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,2*(NLstr(1).alpha(j).sNLk(:,6)).*(NLstr(1).alpha(j).sNLk(:,4)) + (NLstr(1).alpha(j).sNLk(:,6)).^2.*(NLstr(1).alpha(j).sNLk(:,1)) + (NLstr(1).alpha(j).sNLk(:,7)).*(NLstr(1).alpha(j).sNLk(:,2)),'LineWidth',0.6*line_width)
end
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$2 \dot \theta_l \dot y_n + \dot \theta_l^2 x_n + \ddot \theta_l y_n$ [m/s$^2$]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,3)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,-2*(NLstr(1).alpha(j).sNLk(:,6)).*(NLstr(1).alpha(j).sNLk(:,3)) + (NLstr(1).alpha(j).sNLk(:,6)).^2.*(NLstr(1).alpha(j).sNLk(:,2)) - (NLstr(1).alpha(j).sNLk(:,7)).*(NLstr(1).alpha(j).sNLk(:,1)),'LineWidth',0.6*line_width)
end
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$-2 \dot \theta_l \dot x_n + \dot \theta_l^2 y_n - \ddot \theta_l x_n$ [m/s$^2$]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,(NLstr(1).alpha(j).sNLk(:,5)),'LineWidth',0.6*line_width)
end
plot(time,thddMag*th,'--k')
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\theta_l$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(NLstr(1).alpha(j).tNLk,(atan2(NLstr(1).alpha(j).sNLk(:,2),NLstr(1).alpha(j).sNLk(:,1))),'LineWidth',0.6*line_width)
end
%xlim([0 Te])
line([0 2*Te],[-pi -pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
line([0 2*Te],[ pi  pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
% line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_thl_MSD','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_thl_MSD','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

%%
fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x'],'Interpreter', 'latex')
subplot(1,2,1)
k_index = 6;
[maxValue, maxIndex] = max(1000*NLstr(1).alpha(k_index).etaNLk);
indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
hold on
grid on
box on
for i = 2:length(alpha_range)
plot(NLstr(i).alpha(k_index).tNLk,1000*NLstr(i).alpha(k_index).etaNLk,'LineWidth',0.6*line_width)
end
% xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$\alpha_{n} =$',string(num2cell(alpha_range(2:end)))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,['$\mathrm{NL_{vd}} - k = $', num2str(k_range(k_index)), ' - w = ', num2str(w)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
zoomAxes = axes('position',[.30 .20 .15 .25]);
hold on
grid on
box on
for i = 2:length(alpha_range)
plot(NLstr(i).alpha(k_index).tNLk(indexOfInterest),1000*NLstr(i).alpha(k_index).etaNLk(indexOfInterest),'LineWidth',0.6*line_width)
end
% zoomAxes.YLim = 10*[floor(maxValue/10) round(maxValue/10)];

subplot(1,2,2)
% k_index = 6;
% [maxValue, maxIndex] = max(1000*NLstr(1).alpha(k_index).etaNLk);
% indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
hold on
grid on
box on
for i = 1:length(k_range)
plot(NLstr(1).alpha(i).tNLk,1000*NLstr(1).alpha(i).etaNLk,'LineWidth',0.6*line_width)
end
% xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,['$\mathrm{NL_{vd}} - \alpha_n = $', num2str(alpha_range(1))],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

%%
% th_circle = linspace(0,2*pi,361);
% phi_n = atan2(NLstr(3).alpha(4).sNLk(:,2),NLstr(3).alpha(4).sNLk(:,1));
% th_l = NLstr(3).alpha(4).sNLk(:,5);
% th_l_spline = spline(NLstr(3).alpha(4).tNLk,th_l,tNLp);
% 
% figure()
% hold on
% grid on
% box on 
% axis equal
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% plot(1000*R*cos(th_circle),1000*R*sin(th_circle),'LineWidth',0.6*line_width)
% for i = 1:length(NLstr(3).alpha(4).tNLk)
%     sloshMass = plot(1000*NLstr(3).alpha(4).sNLk(i,1),1000*NLstr(3).alpha(4).sNLk(i,2),'--or');
%     sloshPlane = line([0 1000*R*cos(phi_n(i))], [0 1000*R*sin(phi_n(i))],'Color','black');
%     sloshAngle = text(0,0,num2str(rad2deg(phi_n(i))));
%     pause(0.1)
%     delete(sloshMass)
%     delete(sloshPlane)
%     delete(sloshAngle)
% end
% 
% % rho_n = sqrt(sNL(:,1).^2 + sNL(:,2).^2);

