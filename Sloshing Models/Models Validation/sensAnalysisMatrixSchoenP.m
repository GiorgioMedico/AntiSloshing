%--------------------------------------------------------------------------
% Sensistivity Analysis for Sloshing-model Validation (MATRIX) - Pendulum
%
%    Author:     Roberto Di Leva
%    Email:      roberto.dileva@unibo.it 
%    Date:       February 2025
%--------------------------------------------------------------------------
clear all
close all
clc

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
ws_path = fullfile('..', '..','..');
kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");

% addpath(genpath(kin_path));
% addpath(genpath(odes_path));


addpath('Comau_Kinematics');
% addpath('Pendulum');

path_type   = 'LE';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
Te          = 4.8;
Dz_max      = 0.6;
Delta_start_index = -5;

thddMag = 1;
EOMtype = 'NL';

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
animation = 0;

esp_k_min = -6;
esp_k_max = -1;
% esp_k_min = -4;
% esp_k_max = 1;


k_range = logspace(esp_k_min, esp_k_max, esp_k_max - esp_k_min + 1);
% k_range = logspace(esp_k_min, esp_k_max, 10*(esp_k_max - esp_k_min) + 1);

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
fig_name = strcat('Motions_10_10_2024/newFormulation/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
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

gammaPL = (2*h*mn)/(rho*V*tanh(csi11*h/R))*sqrt(csi11^2-1);
gammaPNL = (h*mn*csi11)/(rho*V*tanh(csi11*h/R))*sqrt(2);

for i = 1:length(k_range)
    
    PNLstr(i).k = k_range(i);
    [tPNLk,sPNLk] = ode45(@(t,s)odeSchoenP(t,s,ln,k_range(i),zitan,mn,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan,wn,EOMtype), tspan, S0);
    % [tPNLk,sPNLk] = ode45(@(t,s)odePYX_rotZ(t,s,wn,zitan,g,k_range(i),J,time,rEddy,-rEddx,rEddz,thdd,EOMtype), tspan, S0);

    etaPNLk_bessel = zeros(1,length(tPNLk));
    etaPNLk_planar = zeros(1,length(tPNLk));
    etaPNLk_besselMSD = zeros(1,length(tPNLk));
    etaPNLk_planarMSD = zeros(1,length(tPNLk));
    etaPNLk = zeros(1,length(tPNLk));
    for j=1:length(tPNLk)
        etaPNLk_bessel(j) = gammaPNL*sqrt(1 - cos(sPNLk(j,2)).*cos(sPNLk(j,1)));
        etaPNLk_planar(j) = gammaPL*sqrt(1 - cos(sPNLk(j,2)).*cos(sPNLk(j,1)));
        etaPNLk_besselMSD(j) = gammaNL*ln*sqrt(sin(sPNLk(j,1)).^2.*cos(sPNLk(j,2)).^2 + sin(sPNLk(j,2)).^2);
        etaPNLk_planarMSD(j) = gammaL*ln*sqrt(sin(sPNLk(j,1)).^2.*cos(sPNLk(j,2)).^2 + sin(sPNLk(j,2)).^2);
        etaPNLk(j) = getSloshHeight(sPNLk(j,1),sPNLk(j,2),R);
    end
    phixk  = sPNLk(:,2);
    phiyk  = sPNLk(:,1);
    phixdk = sPNLk(:,4);
    phiydk = sPNLk(:,3);
    thl    = sPNLk(:,5);
    thld   = sPNLk(:,6);
    thldd  = sPNLk(:,7);
    xn = ln*sin(phiyk).*cos(phixk);
    yn = ln*sin(phixk);

    if strcmp(EOMtype,'NL')
        termphiy = cos(phiyk).*tan(phixk).*thldd + cos(phiyk).*sin(phiyk).*thld.^2 + 2*cos(phiydk).*thld.*phixdk;
        termphix = -sin(phiyk).*thldd + cos(phixk).*cos(phiyk).^2.*sin(phixk).*thld.^2 - 2*cos(phixdk).^2.*cos(phiydk).*thld.*phiydk;
    elseif strcmp(EOMtype,'L')
        termphiy = (phixk).*thldd + (phiyk).*thld.^2 + 2*thld.*phixdk;
        termphix = -(phiyk).*thldd + (phixk).*thld.^2 - 2*thld.*phiydk;
    end


    PNLstr(i).etaPNLk_bessel = etaPNLk_bessel;
    PNLstr(i).etaPNLk_planar = etaPNLk_planar;
    PNLstr(i).etaPNLk_besselMSD = etaPNLk_besselMSD;
    PNLstr(i).etaPNLk_planarMSD = etaPNLk_planarMSD;
    PNLstr(i).etaPNLk = etaPNLk;
    PNLstr(i).tPNLk = tPNLk;
    PNLstr(i).sPNLk = sPNLk;
    PNLstr(i).phixk = phixk;
    PNLstr(i).phiyk = phiyk;
    PNLstr(i).phixdk = phixdk;
    PNLstr(i).phiydk = phiydk;
    PNLstr(i).thl = thl;
    PNLstr(i).thld = thld;
    PNLstr(i).thldd = thldd;
    PNLstr(i).xn = xn;
    PNLstr(i).yn = yn;
    PNLstr(i).termphiy = termphiy;
    PNLstr(i).termphix = termphix;

    if animation
        % Load data (assuming sPNLk and tPNLk are already in the workspace)
        phi_y = sPNLk(:,1);  % Rotation about Y-axis
        phi_x = sPNLk(:,2);  % Rotation about X-axis
        ln = 1;  % Length of the pendulum (modify as needed)
        t = tPNLk;  % Time vector
        
        
        % Compute Cartesian coordinates
        x = ln * sin(phi_y) .* cos(phi_x);
        y = ln * sin(phi_x);
        z = -ln * cos(phi_x) .* cos(phi_y);
    
        % Resample accelerations to match the number of time steps in tPNLk
        Nt = length(t);  % Number of time steps in t
        Na = length(rEddx);  % Number of steps in acceleration data
        time_acc = linspace(t(1), time(end), Na);  % Time vector for acceleration data
        
        % Interpolate accelerations to match tPNLk, ensuring they become 0 after time_acc(end)
        rEddx_resampled = interp1([time_acc, t(end)], [rEddx, 0], t, 'linear', 'extrap');
        rEddy_resampled = interp1([time_acc, t(end)], [rEddy, 0], t, 'linear', 'extrap');
        rEddz_resampled = interp1([time_acc, t(end)], [rEddz, 0], t, 'linear', 'extrap');
    
        
        % Create figure
        figure;
        hold on;
        grid on;
        axis equal;
        view(3);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        xlim([-ln ln]); ylim([-ln ln]); zlim([-ln ln/2]);
        title(sprintf('Spherical Pendulum Motion - K: %f', k_range(i)));
        pendulum_line = plot3([0 x(1)], [0 y(1)], [0 z(1)], 'r-', 'LineWidth', 2); % Rod
        pendulum_mass = plot3(x(1), y(1), z(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Mass
        % Vertical dashed line through pivot
        pivot_line = plot3([0 0], [0 0], [0 -ln], 'k--', 'LineWidth', 1);
    
        % Reference frame at pivot (X, Y, Z arrows)
        quiver3(0, 0, 0, ln/2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X-axis (red)
        quiver3(0, 0, 0, 0, ln/2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y-axis (green)
        quiver3(0, 0, 0, 0, 0, ln/2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z-axis (blue)
        motion_arrow = quiver3(0, 0, 0, rEddy_resampled(1)/5, -rEddx_resampled(1)/5, rEddz_resampled(1)/5, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
        
        % Animation loop
        for j = 1:length(t)
            % Update pendulum position
            set(pendulum_line, 'XData', [0 x(j)], 'YData', [0 y(j)], 'ZData', [0 z(j)]);
            set(pendulum_mass, 'XData', x(j), 'YData', y(j), 'ZData', z(j));  
    
            % Update motion direction arrow
            set(motion_arrow, 'XData', 0, 'YData', 0, 'ZData', 0, ...
                          'UData', rEddy_resampled(j)/5, 'VData', -rEddx_resampled(j)/5, 'WData', rEddz_resampled(j)/5);
            
            % Pause for smooth animation (adjust for real-time speed)
            pause(0.01);
        end
        
        hold off;
    end
end




%% Sloshing-Height Formulation (3D-NL)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with paraboloic term 
[tPNL,sPNL] = ode45(@(t,s)odeSchoenP(t,s,ln,k,zitan,mn,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,alphan,wn,EOMtype), tspan, S0);
for j=1:length(tPNL)
    etaPNL_bessel(j) = gammaPNL*sqrt(1 - cos(sPNL(j,2)).*cos(sPNL(j,1)));
    etaPNL_planar(j) = gammaPL*sqrt(1 - cos(sPNL(j,2)).*cos(sPNL(j,1)));
    etaPNL_besselMSD(j) = gammaNL*ln*sqrt(sin(sPNL(j,1)).^2.*cos(sPNL(j,2)).^2 + sin(sPNL(j,2)).^2);
    etaPNL_planarMSD(j) = gammaL*ln*sqrt(sin(sPNL(j,1)).^2.*cos(sPNL(j,2)).^2 + sin(sPNL(j,2)).^2);
    etaPNL(j) = getSloshHeight(sPNL(j,1),sPNL(j,2),R);
end
xnPNL = ln*sin(sPNL(:,1)).*cos(sPNL(:,2));
ynPNL = ln*sin(sPNL(:,2));


[tNL,sNL] = ode45(@(t,s)odeSchoenMSD(t,s,kn,k,zitan,mn,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,alphan,2,EOMtype), tspan, S0);
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
plot(PNLstr(j).tPNLk,rad2deg(PNLstr(j).phixk),'LineWidth',0.6*line_width)
end
plot(tPNL,rad2deg(sPNL(:,2)),'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_x [^\circ]$','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,2)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,rad2deg(PNLstr(j).phiyk),'LineWidth',0.6*line_width)
end
plot(tPNL,rad2deg(sPNL(:,1)),'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_y [^\circ]$','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{PNL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,3)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,(1000*PNLstr(j).xn).^2 + (1000*PNLstr(j).yn).^2,'LineWidth',0.6*line_width)
end
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_n^2 + y_n^2$ [mm$^2$]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,['$\mathrm{P',EOMtype,'_{vd}}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,1000*PNLstr(j).etaPNLk_bessel,'LineWidth',0.6*line_width)
end
plot(tPNL,1000*etaPNL_bessel,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta_{Bessel}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,1000*PNLstr(j).etaPNLk_planar,'LineWidth',0.6*line_width)
end
plot(tPNL,1000*etaPNL_planar,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline \eta_{Planar}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,6)
hold on
grid on
box on 
plot(tPNL,(1000*xnPNL).^2+(1000*ynPNL).^2,'LineWidth',0.6*line_width)
plot(tNL,(1000*sNL(:,1)).^2+(1000*sNL(:,2)).^2,'--k')
%xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$x_n^2 + y_n^2$ [mm$^2$]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend(['$\mathrm{',EOMtype,'_{P}}$'],['$\mathrm{',EOMtype,'_{MSD}}$'],'Fontsize',legend_size,'interpreter', 'latex','Location','best');
title(leg,'$\ddot \theta_0 = 0$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');


if save_fig
    save_name = strcat(fig_name,'_k_influence_phix_phiy_P',EOMtype,'.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    % save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_xnyn_MSD','.pdf');
    % print('-dpdf', '-fillpage', save_name)
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
plot(PNLstr(j).tPNLk,(PNLstr(j).sPNLk(:,6)),'LineWidth',0.6*line_width)
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
plot(PNLstr(j).tPNLk,(PNLstr(j).sPNLk(:,5)),'LineWidth',0.6*line_width)
end
plot(time,thddMag*th,'--k')
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\theta_l$ [rad]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,PNLstr(j).termphix,'LineWidth',0.6*line_width)
end
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_x$ term','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
for j = 1:length(k_range)
plot(PNLstr(j).tPNLk,PNLstr(j).termphiy,'LineWidth',0.6*line_width)
end
% %xlim([0 Te])
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_y$ term','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,'$\mathrm{NL_{vd}}$','interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat(fig_name,'k_influence_thl_P',EOMtype,'.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    % save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_influence_thl_MSD','.pdf');
    % print('-dpdf', '-fillpage', save_name)
end

%%
% fig = figure()
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
% sgtitle([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x'],'Interpreter', 'latex')
% subplot(1,2,1)
% k_index = 6;
% [maxValue, maxIndex] = max(1000*NLstr(1).alpha(k_index).etaNLk);
% indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
% hold on
% grid on
% box on
% for i = 2:length(alpha_range)
% plot(NLstr(i).alpha(k_index).tNLk,1000*NLstr(i).alpha(k_index).etaNLk,'LineWidth',0.6*line_width)
% end
% % xlim([0 Te])
% xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$\alpha_{n} =$',string(num2cell(alpha_range(2:end)))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,['$\mathrm{NL_{vd}} - k = $', num2str(k_range(k_index)), ' - w = ', num2str(w)],'interpreter', 'latex')
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% zoomAxes = axes('position',[.30 .20 .15 .25]);
% hold on
% grid on
% box on
% for i = 2:length(alpha_range)
% plot(NLstr(i).alpha(k_index).tNLk(indexOfInterest),1000*NLstr(i).alpha(k_index).etaNLk(indexOfInterest),'LineWidth',0.6*line_width)
% end
% % zoomAxes.YLim = 10*[floor(maxValue/10) round(maxValue/10)];
% 
% subplot(1,2,2)
% % k_index = 6;
% % [maxValue, maxIndex] = max(1000*NLstr(1).alpha(k_index).etaNLk);
% % indexOfInterest = maxIndex-20:maxIndex+20; % range of t near perturbation
% hold on
% grid on
% box on
% for i = 1:length(k_range)
% plot(NLstr(1).alpha(i).tNLk,1000*NLstr(1).alpha(i).etaNLk,'LineWidth',0.6*line_width)
% end
% % xlim([0 Te])
% xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$\overline \eta$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(strcat('$k_{vd} =$',string(num2cell(k_range))),'Fontsize',legend_size,'interpreter', 'latex','Location','best');
% title(leg,['$\mathrm{NL_{vd}} - \alpha_n = $', num2str(alpha_range(1))],'interpreter', 'latex')
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% 
% if save_fig
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
%     save_name = strcat([fig_name,'_R',num2str(1000*R),'_',num2str(thddMag),'x_'],'k_alpha_influence_eta_MSD','.pdf');
%     print('-dpdf', '-fillpage', save_name)
% end

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

