%--------------------------------------------------------------------------
% Sensistivity Analysis for Sloshing-model Validation (MATRIX)
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

path_type   = 'Tilt_TRD';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
Te          = 2.5;
A_psi       = deg2rad(30);
Dz_max      = 0.6;
Delta_start_index = -0;

R = 0.049;
h = 0.08;
% [g, rho, m_tot, V, csi11, zitan, mn, kn, cs, alphan, l, J, k, wn] = Parameters(R, h);
[g, rho, m_tot, V, csi11, zitan, mn, kn, cn, alphan, ln, Ln, J, k, wn] = ParametersP(R, h);

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 0;
save_fig   = 0;

esp_k_min = -6;
esp_k_max = -1;

k_range = logspace(esp_k_min, esp_k_max, esp_k_max - esp_k_min + 1);
% alpha_range = [0.58, linspace(1/100,10,11)];
alpha_range = [0.58, linspace(0,1,5)];
w = 3;
a  = h/R;
% hn = h*(0.5-(2/(csi11*a))*tanh(csi11*0.5*a));

hn = h/2 - R/csi11*tanh(csi11*h/R);

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
fig_name = strcat('Motions_10_10_2024/Sens_Analysis/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
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

%% (vd) Sloshing-Height Computation with Varying k and alpha_n
tspan = [0 2*Te];
S0 = [0 0 0 0];

gammaNL = (h*mn*csi11^2)/(rho*V*R);

for j = 1:length(alpha_range)
    
    NLstr(j).alpha = alpha_range(j);
    [tNLt,sNLt] = ode45(@(t,s)odeTiltMSD(t,s,kn,k,zitan,mn,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,hn,g,alpha_range(j)/R^(2*w-2),w,'NL'), tspan, S0);
    etaNLt = gammaNL*(sNLt(:,1).^2 + sNLt(:,2).^2).^0.5;

    NLstr(j).tNLt = tNLt;
    NLstr(j).sNLt = sNLt;
    NLstr(j).etaNLt = etaNLt;

end


%% Figures: Influence of k
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;

fig = figure('Visible','off')
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R)],'Interpreter', 'latex')
subplot(2,3,1)
hold on
grid on
box on 
plot(NLstr(1).tNLt,1000*(NLstr(1).sNLt(:,1)),'LineWidth',0.6*line_width)
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
plot(NLstr(1).tNLt,1000*(NLstr(1).sNLt(:,2)),'LineWidth',0.6*line_width)
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
plot(NLstr(1).tNLt,((1000*NLstr(1).sNLt(:,1)).^2),'LineWidth',0.6*line_width)
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
plot(NLstr(1).tNLt,((1000*NLstr(1).sNLt(:,2)).^2),'LineWidth',0.6*line_width)
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
plot(NLstr(1).tNLt,((1000*NLstr(1).sNLt(:,1)).^2)+((1000*NLstr(1).sNLt(:,2)).^2),'LineWidth',0.6*line_width)
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
plot(NLstr(1).tNLt,(atan2(NLstr(1).sNLt(:,2),NLstr(1).sNLt(:,1))),'LineWidth',0.6*line_width)
line([0 2*Te],[-pi -pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
line([0 2*Te],[ pi  pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,['$\mathrm{NL_{vd}} - \alpha_n = $', num2str(alpha_range(1))],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

% if save_fig
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_xnyn_MSD','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_xnyn_MSD','.pdf');
%     print('-dpdf', '-fillpage', save_name)
% end



%%

fig = figure('Visible','off')
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R)],'Interpreter', 'latex')

subplot(2,3,5)
hold on
grid on
box on 
plot(NLstr(1).tNLt,(atan2(NLstr(1).sNLt(:,2),NLstr(1).sNLt(:,1))),'LineWidth',0.6*line_width)
%xlim([0 Te])
line([0 2*Te],[-pi -pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
line([0 2*Te],[ pi  pi],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
% line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

% if save_fig
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_thl_MSD','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_thl_MSD','.pdf');
%     print('-dpdf', '-fillpage', save_name)
% end

%%
fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle([fig_name,'_R',num2str(1000*R)],'Interpreter', 'latex')
subplot(1,2,1)
k_index = 6;
[maxValue, maxIndex] = max(1000*NLstr(1).etaNLt);
indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
hold on
grid on
box on
for i = 2:length(alpha_range)
plot(NLstr(i).tNLt,1000*NLstr(i).etaNLt,'LineWidth',0.6*line_width)
end
% xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$\alpha_{n} =$',string(num2cell(alpha_range(2:end)))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,['$\mathrm{NL_{t}} $', ' - w = ', num2str(w)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
zoomAxes = axes('position',[.30 .20 .15 .25]);
hold on
grid on
box on
for i = 2:length(alpha_range)
plot(NLstr(i).tNLt(indexOfInterest),1000*NLstr(i).etaNLt(indexOfInterest),'LineWidth',0.6*line_width)
end
% zoomAxes.YLim = 10*[floor(maxValue/10) round(maxValue/10)];

subplot(1,2,2)
% k_index = 6;
% [maxValue, maxIndex] = max(1000*NLstr(1).alpha(k_index).etaNLt);
% indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
hold on
grid on
box on
plot(NLstr(1).tNLt,1000*NLstr(1).etaNLt,'LineWidth',0.6*line_width)
% xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$\alpha_n =$',string(num2cell(alpha_range(1)))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.pdf');
    print('-dpdf', '-fillpage', save_name)
end


