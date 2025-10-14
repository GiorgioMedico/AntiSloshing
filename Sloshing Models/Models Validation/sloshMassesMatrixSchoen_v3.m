%--------------------------------------------------------------------------
% # of Sloshing masses for Sloshing-model Validation (MATRIX) v2
%
%    Author:     Roberto Di Leva
%    Email:      roberto.dileva@unibo.it 
%    Date:       March 2025
%--------------------------------------------------------------------------
clear all
close all
clc

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
addpath('Comau_Kinematics');
% addpath('Pendulum');

path_type   = 'TRD';
dim_type    = '3D';
motion_type = [path_type,'_',dim_type];
Te          = 2.8;
Dz_max      = 0.6;
Delta_start_index = -1;
sloshing_masses = [1 2 3];
motion_type_new = 'Sg3';
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

esp_k_min = -6;
% esp_k_max = -1;
esp_k_max = -1;

k_range = logspace(esp_k_min, esp_k_max, 2*(esp_k_max - esp_k_min) + 1);
% alpha_range = [0.42, 0.50, 0.58, 0.66, 0.74];
alpha_sota = [0, 0.58, 1];
w_sota = [1, 2, 1];
alpha_range = linspace(0,1,5);
w_range = [1 2 3];
alphan = 0;

k_ = 10^esp_k_max;
w = 2;

EOMtype = 'NL';

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
fig_name = strcat('Motions_10_10_2024/Comparison/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
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

%% (vd) Sloshing-Height Computation with Varying k and sloshing masses
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];

gammaL1 = (4*h*m1)/(rho*V*R);
gammaL2 = (4*h*m2)/(rho*V*R);
gammaL3 = (4*h*m3)/(rho*V*R);

gammaNL1 = (h*m1*csi11^2)/(rho*V*R);
gammaNL2 = (h*m2*csi12^2)/(rho*V*R);
gammaNL3 = (h*m3*csi13^2)/(rho*V*R);

time_spline = linspace(0,2*Te,2*Te*500+1);
psi_spline = linspace(0,2*pi,361);
r_spline = linspace(0,R,50); 
        
tic 
[tNLk1,sNLk1] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k_,zita1,m1,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan/R^2,w,EOMtype), tspan, S0);
toc
tic
etaNLk1 = gammaNL1*(sNLk1(:,1).^2 + sNLk1(:,2).^2).^0.5;
toc
phiNLk1 = atan2(sNLk1(:,2),sNLk1(:,1));
xnNLk1 = sNLk1(:,1);
ynNLk1 = sNLk1(:,2);

tic
[tNLk2,sNLk2] = ode45(@(t,s)odeSchoenMSD(t,s,k2,k_,zita2,m2,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan/R^2,w,'NL'), tspan, S0);
toc
etaNLk2 = gammaNL2*(sNLk2(:,1).^2 + sNLk2(:,2).^2).^0.5;
phiNLk2 = atan2(sNLk2(:,2),sNLk2(:,1));
xnNLk2 = sNLk2(:,1);
ynNLk2 = sNLk2(:,2);

tic
[tNLk3,sNLk3] = ode45(@(t,s)odeSchoenMSD(t,s,k3,k_,zita3,m3,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan/R^2,w,'NL'), tspan, S0);
toc
etaNLk3 = gammaNL3*(sNLk3(:,1).^2 + sNLk3(:,2).^2).^0.5;
phiNLk3 = atan2(sNLk3(:,2),sNLk3(:,1));
xnNLk3 = sNLk3(:,1);
ynNLk3 = sNLk3(:,2);

etaNLk1_spline = spline(tNLk1,etaNLk1,time_spline);
etaNLk2_spline = spline(tNLk2,etaNLk2,time_spline);
etaNLk3_spline = spline(tNLk3,etaNLk3,time_spline);

phiNLk1_spline = spline(tNLk1,phiNLk1,time_spline);
phiNLk2_spline = spline(tNLk2,phiNLk2,time_spline);
phiNLk3_spline = spline(tNLk3,phiNLk3,time_spline);

xnNLk1_spline = spline(tNLk1,xnNLk1,time_spline);
ynNLk1_spline = spline(tNLk1,ynNLk1,time_spline);

for j = 1:length(time_spline)
    
    % eta = etaNLk1_spline(i)*besselj(1,csi11*r_/R)/besselj(1,csi11);

    for i = 1:length(r_spline)
    etaNLk1_psi_time(:,i,j) = etaNLk1_spline(j)*besselj(1,csi11*r_spline(i)/R)/besselj(1,csi11)*cos(psi_spline - phiNLk1_spline(j));
    etaNLk2_psi_time(:,i,j) = etaNLk2_spline(j)*besselj(1,csi12*r_spline(i)/R)/besselj(1,csi12)*cos(psi_spline - phiNLk2_spline(j));
    etaNLk3_psi_time(:,i,j) = etaNLk3_spline(j)*besselj(1,csi13*r_spline(i)/R)/besselj(1,csi13)*cos(psi_spline - phiNLk3_spline(j));

    etaNLk1_psi_time_v2(:,i,j) = gammaNL1*besselj(1,csi11*r_spline(i)/R)/besselj(1,csi11)*(cos(psi_spline)*xnNLk1_spline(j) + sin(psi_spline)*ynNLk1_spline(j));
    % etaNLk1_psi_time_max(j)   = max(etaNLk1_psi_time(:,j));
    % etaNLk12_psi_time_max(j)  = max(etaNLk1_psi_time(:,j)+etaNLk2_psi_time(:,j));
    % etaNLk123_psi_time_max(j) = max(etaNLk1_psi_time(:,j)+etaNLk2_psi_time(:,j)+etaNLk3_psi_time(:,j));

    end

    [etaNLk1_psi_time_max_rows(j,:)  , etaNLk1_max_rows(j,:)  ] = (max(etaNLk1_psi_time(:,:,j)));
    [etaNLk12_psi_time_max_rows(j,:) , etaNLk12_max_rows(j,:) ] = (max(etaNLk1_psi_time(:,:,j)+etaNLk2_psi_time(:,:,j)));
    [etaNLk123_psi_time_max_rows(j,:), etaNLk123_max_rows(j,:)] = (max(etaNLk1_psi_time(:,:,j)+etaNLk2_psi_time(:,:,j)+etaNLk3_psi_time(:,:,j)));
    [etaNLk1_psi_time_max(j),   etaNLk1_max_cols(j)  ] = max(etaNLk1_psi_time_max_rows(j,:));
    [etaNLk12_psi_time_max(j),  etaNLk12_max_cols(j) ] = max(etaNLk12_psi_time_max_rows(j,:));
    [etaNLk123_psi_time_max(j), etaNLk123_max_cols(j)] = max(etaNLk123_psi_time_max_rows(j,:));
    
    etaNLk1_psi_max(j)   = psi_spline(etaNLk1_max_rows(j,etaNLk1_max_cols(j)));
    etaNLk12_psi_max(j)  = psi_spline(etaNLk12_max_rows(j,etaNLk12_max_cols(j)));
    etaNLk123_psi_max(j) = psi_spline(etaNLk123_max_rows(j,etaNLk123_max_cols(j)));

    etaNLk1_psi_max_r(:,j)   = etaNLk1_psi_time(etaNLk1_max_rows(j,etaNLk1_max_cols(j)),:,j);
    etaNLk2_psi_max_r(:,j)   = etaNLk2_psi_time(etaNLk1_max_rows(j,etaNLk1_max_cols(j)),:,j);
    etaNLk3_psi_max_r(:,j)   = etaNLk3_psi_time(etaNLk1_max_rows(j,etaNLk1_max_cols(j)),:,j);
    etaNLk12_psi_max_r(:,j)  = etaNLk1_psi_time(etaNLk12_max_rows(j,etaNLk12_max_cols(j)),:,j) + etaNLk2_psi_time(etaNLk12_max_rows(j,etaNLk12_max_cols(j)),:,j);
    etaNLk123_psi_max_r(:,j) = etaNLk1_psi_time(etaNLk123_max_rows(j,etaNLk123_max_cols(j)),:,j) + etaNLk2_psi_time(etaNLk123_max_rows(j,etaNLk123_max_cols(j)),:,j) + etaNLk3_psi_time(etaNLk123_max_rows(j,etaNLk123_max_cols(j)),:,j);

    [etaNLk1_psi_time_maxR(j), etaNLk1_ind_maxR(j)]     = (max(etaNLk1_psi_time(:,end,j)));
    [etaNLk12_psi_time_maxR(j), etaNLk12_ind_maxR(j)]   = (max(etaNLk1_psi_time(:,end,j)+etaNLk2_psi_time(:,end,j)));
    [etaNLk123_psi_time_maxR(j), etaNLk123_ind_maxR(j)] = (max(etaNLk1_psi_time(:,end,j)+etaNLk2_psi_time(:,end,j)+etaNLk3_psi_time(:,end,j)));
    % etaNLk1_psi_time_R(:,j) = etaNLk1_psi_time(:,end,j);
    % etaNLk1_psi_time_R_v2(:,j) = etaNLk1_psi_time_v2(:,end,j);

end

[max_eta, index_eta] = max(etaNLk1_spline);
% index_eta = 591;
% etaNLk1_r = etaNLk1_psi_time(etaNLk123_max_rows(index_eta,etaNLk123_max_cols(index_eta)),:,index_eta);
etaNLk2_r = etaNLk2_psi_time(etaNLk123_max_rows(index_eta,etaNLk123_max_cols(index_eta)),:,index_eta);
etaNLk3_r = etaNLk3_psi_time(etaNLk123_max_rows(index_eta,etaNLk123_max_cols(index_eta)),:,index_eta);

etaNLk2_phi1max_r = etaNLk2_psi_max_r(:,index_eta);
etaNLk3_phi1max_r = etaNLk3_psi_max_r(:,index_eta);

etaNLk1_r  = etaNLk1_psi_max_r(:,index_eta);
etaNLk12_r  = etaNLk12_psi_max_r(:,index_eta);
etaNLk123_r = etaNLk123_psi_max_r(:,index_eta);

%% P-tan
[tPNLk1,sPNLk1] = ode45(@(t,s)odeSchoenP(t,s,l1,k_,zita1,m1,time,rEddy,-rEddx,rEddz,thdd,J,g,alphan,w1,EOMtype), tspan, S0);
phixPNLk1 = sPNLk1(:,2);
phiyPNLk1 = sPNLk1(:,1);
xnPNLk1 = l1*sin(phiyPNLk1).*cos(phixPNLk1);
ynPNLk1 = l1*sin(phixPNLk1);
phiPNLk1 = atan2(ynPNLk1,xnPNLk1);

for j=1:length(tPNLk1)
     etaPNLk1_tan(j) = getSloshHeight(sPNLk1(j,1),sPNLk1(j,2),R);
end

%% Phasing procedure
%--------------------------------------------------------------------------
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L with paraboloic term 
[tLp,sL] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,alphan,2,'L'), tspan, S0);
gammaL = (4*h*m1)/(rho*V*R);
etaL = gammaL*(sL(:,1).^2 + sL(:,2).^2).^0.5;

clear etaPar
for i = 1:length(tLp)
    if tLp(i)<Te
        thetad(i) = spline(time,thd,tLp(i));
        etaPar(i) = R^2*thetad(i)^2/(4*g);
    else
        etaPar(i) = 0;
    end
end
etaLPar = etaL + etaPar';

%--------------------------------------------------------------------------

Efps = 60; % Framerate experiment video
end_time = 1.5*Te;
[~, emax_index] = max(slosh_heights);
[~, Mmax_index] = max(etaLPar);
peak_time = tLp(Mmax_index);
E_start_index = round(emax_index - peak_time*Efps) + Delta_start_index;
E_slosh_h_cut_1 = slosh_heights(E_start_index+1:E_start_index+(end_time*Efps));
E_slosh_t_cut_1 = slosh_times(E_start_index+1:E_start_index+(end_time*Efps)) - slosh_times(E_start_index+1);

%% Accuracy-index computation
eps_m1 = calcola_errore_picco(1000*etaNLk1_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_m2 = calcola_errore_picco(1000*etaNLk12_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_m3  = calcola_errore_picco(1000*etaNLk123_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_n1 = calcola_errore_picco(1000*(etaNLk1_spline), abs(E_slosh_h_cut_1), 0, 1);
eps_n2 = calcola_errore_picco(1000*(etaNLk1_spline+etaNLk2_spline), abs(E_slosh_h_cut_1), 0, 1);
eps_n3  = calcola_errore_picco(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline), abs(E_slosh_h_cut_1), 0, 1);

[eps_m1, eps_m2, eps_m3; eps_n1, eps_n2, eps_n3]

sigma_m1 = media_errore(1000*etaNLk1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_m2 = media_errore(1000*etaNLk12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_m3 = media_errore(1000*etaNLk123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n1_old = media_errore(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n2_old = media_errore(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n3_old = media_errore(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

sigma_n1 = errore_integrale(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n2 = errore_integrale(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n3 = errore_integrale(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n1_norm = errore_integrale(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);
sigma_n2_norm = errore_integrale(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);
sigma_n3_norm = errore_integrale(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);

[sigma_m1, sigma_m2, sigma_m3; sigma_n1, sigma_n2, sigma_n3]

epsR_m1 = calcola_errore_picco_esaurimento(1000*etaNLk1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_m2 = calcola_errore_picco_esaurimento(1000*etaNLk12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_m3 = calcola_errore_picco_esaurimento(1000*etaNLk123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n1 = calcola_errore_picco_esaurimento(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n2 = calcola_errore_picco_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n3 = calcola_errore_picco_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

[epsR_m1, epsR_m2, epsR_m3; epsR_n1, epsR_n2, epsR_n3]

sigmaR_m1 = media_errore_esaurimento(1000*etaNLk1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_m2 = media_errore_esaurimento(1000*etaNLk12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_m3 = media_errore_esaurimento(1000*etaNLk123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n1_old = media_errore_esaurimento(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n2_old = media_errore_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n3_old = media_errore_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

sigmaR_n1 = errore_integrale_esaurimento(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n2 = errore_integrale_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n3 = errore_integrale_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n1_norm = errore_integrale_esaurimento(1000*(etaNLk1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);
sigmaR_n2_norm = errore_integrale_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);
sigmaR_n3_norm = errore_integrale_esaurimento(1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);

[sigmaR_m1, sigmaR_m2, sigmaR_m3; sigmaR_n1, sigmaR_n2, sigmaR_n3]
%% Graphics
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;

max_slosh_mod = max(etaLPar*1000);
max_slosh = max(max_slosh_mod,max(E_slosh_h_cut_1));
Ylim      = (floor(max_slosh/5)+1)*5 + 10;


fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle(fig_name,'Interpreter', 'latex')
subplot(2,3,1)
title(motion_type_new,'interpreter', 'latex')
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim-10])
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.7*line_width)
plot(time_spline,1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),'--','Color',"#77AC30",'LineWidth',0.7*line_width)
plot(tPNLk1,1000*etaPNLk1_tan,':','Color','#7E2F8E','LineWidth',0.7*line_width)
line([Te Te],[0 Ylim-10],'Color','k','LineStyle','--')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline{\eta}$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('Exp','MDP','P-tan','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,2)
hold on
grid on
box on 
xlim([0 1.0*Te])
% ylim([0 Ylim])
plot(time_spline,rad2deg(etaNLk1_psi_max),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(etaNLk12_psi_max),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(etaNLk123_psi_max),'LineWidth',0.6*line_width)
% plot(time_spline,rad2deg(etaNLk123_psi_max),'--k')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_{max}$ [deg]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$','- $k_{vd} =$',num2str(k_)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
grid on
box on 
% xlim([0 1.5*Te])
% ylim([0 Ylim])
plot(1000*r_spline,1000*etaNLk1_r,'LineWidth',0.6*line_width)
% plot(1000*r_spline,1000*(etaNLk1_r+etaNLk2_r),'LineWidth',0.6*line_width)
% plot(1000*r_spline,1000*(etaNLk1_r+etaNLk2_r+etaNLk3_r),'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*etaNLk12_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*etaNLk123_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk2_r),'--','Color',"#D95319",'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk3_r),'--','Color',"#EDB120",'LineWidth',0.6*line_width)
xlabel('$r$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('${\eta}(\phi^*,r,t^*)$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$','- $k_{vd} =$',num2str(k_)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim])
plot(time_spline,1000*etaNLk1_spline,'LineWidth',0.6*line_width)
plot(time_spline,1000*(etaNLk1_spline+etaNLk2_spline),'LineWidth',0.6*line_width)
plot(time_spline,1000*(etaNLk1_spline+etaNLk2_spline+etaNLk3_spline),'LineWidth',0.6*line_width)
% plot(time_spline,1000*etaPNLk1_spline,'--k')
% plot(time_spline,1000*etaNLk1_psi_time_maxR,'--k')
% plot(time_spline,1000*etaNLk12_psi_time_maxR,'--k')
% plot(time_spline,1000*etaNLk123_psi_time_maxR,'--k')
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),':','LineWidth',0.6*line_width,'Color','#7E2F8E')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\sum_{k=1}^n\overline{\eta}_k$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$n=1$','$n=2$','$n=3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$','- $k_{vd} =$',num2str(k_)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,5)
hold on
grid on
box on 
xlim([0 1.0*Te])
% ylim([0 Ylim])
plot(time_spline,rad2deg(wrapTo2Pi(phiNLk1_spline)),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(wrapTo2Pi(phiNLk2_spline)),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(wrapTo2Pi(phiNLk3_spline)),'LineWidth',0.6*line_width)
% plot(time_spline,rad2deg(etaNLk123_psi_max),'--k')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [deg]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$n=1$','$n=2$','$n=3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$','- $k_{vd} =$',num2str(k_)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,6)
hold on
grid on
box on 
% xlim([0 1.5*Te])
% ylim([0 Ylim])
plot(1000*r_spline,1000*etaNLk1_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk1_r+etaNLk2_phi1max_r),'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk1_r+etaNLk2_phi1max_r+etaNLk3_phi1max_r),'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk2_phi1max_r),'--','Color',"#D95319",'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLk3_phi1max_r),'--','Color',"#EDB120",'LineWidth',0.6*line_width)
xlabel('$r$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('${\eta}(\phi^*,r,t^*)$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$','- $k_{vd} =$',num2str(k_)],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

if save_fig
    save_name = strcat(fig_name,'_MDP_vs_P-tan','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'_MDP_vs_P-tan','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

if save_csv
    % First dataset
    T1 = table(E_slosh_t_cut_1(:), abs(E_slosh_h_cut_1(:)), ...
        'VariableNames', {'Time', 'Eta_exp'});
    writetable(T1, fullfile('csv',strcat(motion_type_new, '_exp.csv')));
    
    % Second dataset
    etaNL_total = 1000 * (etaNLk1_spline(:) + etaNLk2_spline(:) + etaNLk3_spline(:));
    T2 = table(time_spline(:), etaNL_total, ...
        'VariableNames', {'Time', 'Eta_PMD'});
    writetable(T2, fullfile('csv', strcat(motion_type_new, '_PMD.csv')));
    
    % Third dataset
    T3 = table(tPNLk1(:), 1000 * etaPNLk1_tan(:), ...
        'VariableNames', {'Time', 'Eta_PEN'});
    writetable(T3, fullfile('csv', strcat(motion_type_new, '_PEN.csv')));
end

%%
fig = figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
sgtitle(fig_name,'Interpreter', 'latex')
subplot(2,3,1)
plot(0, 0)
axis on 
headers = {'Index - m', '', '', ''};
data = {'$\epsilon$ [\%]', eps_m1, eps_m2, eps_m3; 
        '$\sigma$ [\%]',   sigma_m1, sigma_m2, sigma_m3; 
        '$\epsilon_r$ [\%]', epsR_m1, epsR_m2, epsR_m3; 
        '$\sigma_r$ [\%]', sigmaR_m1, sigmaR_m2, sigmaR_m3};
xPos = 0.1;
yPos = 0.9;  
headerFormat = '%-10s %-10s %-10s %-10s';  % fixed width for each column
text(xPos, yPos, sprintf(headerFormat, headers{1}, headers{2}, headers{3}, headers{4}), ...
    'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 'black','Interpreter','latex');
yPos = yPos - 0.1;
for i = 1:size(data, 1)
    % Display the variable (string, left-aligned)
    text(xPos, yPos, sprintf('%-10s', data{i, 1}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 'black','Interpreter','latex');
    % Display the first numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.15, yPos, sprintf('%-10.2f', data{i, 2}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 	"black",'Interpreter','latex');    
    % Display the second numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.30, yPos, sprintf('%-10.2f', data{i, 3}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', "black",'Interpreter','latex'); 
    % Display the third numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.45, yPos, sprintf('%-10.2f', data{i, 4}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', "black",'Interpreter','latex');
    % Move down for the next row
    yPos = yPos - 0.1;
end

subplot(2,3,2)
plot(0, 0)
axis on 
headers = {'Index - n', '', '', ''};
data = {'$\epsilon$ [\%]', eps_n1, eps_n2, eps_n3; 
        '$\sigma$ [mm]',   sigma_n1, sigma_n2, sigma_n3; 
        '$\overline{\sigma}$ [\%]',   sigma_n1_norm, sigma_n2_norm, sigma_n3_norm;
        '$\sigma_r$ [mm]', sigmaR_n1, sigmaR_n2, sigmaR_n3; 
        '$\overline{\sigma}_r$ [\%]', sigmaR_n1_norm, sigmaR_n2_norm, sigmaR_n3_norm};
xPos = 0.1;
yPos = 0.9;  
headerFormat = '%-10s %-10s %-10s %-10s';  % fixed width for each column
text(xPos, yPos, sprintf(headerFormat, headers{1}, headers{2}, headers{3}, headers{4}), ...
    'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 'black','Interpreter','latex');
yPos = yPos - 0.1;
for i = 1:size(data, 1)
    % Display the variable (string, left-aligned)
    text(xPos, yPos, sprintf('%-10s', data{i, 1}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 'black','Interpreter','latex');
    % Display the first numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.20, yPos, sprintf('%-10.2f', data{i, 2}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', 	"black",'Interpreter','latex');    
    % Display the second numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.35, yPos, sprintf('%-10.2f', data{i, 3}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', "black",'Interpreter','latex'); 
    % Display the third numerical value (formatted with 2 decimal places, aligned)
    text(xPos + 0.50, yPos, sprintf('%-10.2f', data{i, 4}), ...
        'FontSize', 12, 'FontName', 'Courier', 'VerticalAlignment', 'top', 'Units', 'normalized', 'Color', "black",'Interpreter','latex');
    % Move down for the next row
    yPos = yPos - 0.1;
end

subplot(2,3,4)
hold on
grid on
box on 
xlim([0 1.5*Te])
plot(tNLk1,1000*xnNLk1,'LineWidth',0.6*line_width)
plot(tNLk1,1000*ynNLk1,'LineWidth',0.6*line_width)
plot(tNLk1,1000*sqrt(xnNLk1.^2+ynNLk1.^2),'--k','LineWidth',0.3*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('[mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$x_1$','$y_1$','$\sqrt{x_1^2 + y_1^2}$','Fontsize',legend_size,'interpreter', 'latex','Location','northeast','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
xlim([0 1.5*Te])
plot(tNLk2,1000*xnNLk2,'LineWidth',0.6*line_width)
plot(tNLk2,1000*ynNLk2,'LineWidth',0.6*line_width)
plot(tNLk2,1000*sqrt(xnNLk2.^2+ynNLk2.^2),'--k','LineWidth',0.3*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('[mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$x_2$','$y_2$','$\sqrt{x_2^2 + y_2^2}$','Fontsize',legend_size,'interpreter', 'latex','Location','northeast','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,6)
hold on
grid on
box on 
xlim([0 1.5*Te])
plot(tNLk3,1000*xnNLk3,'LineWidth',0.6*line_width)
plot(tNLk3,1000*ynNLk3,'LineWidth',0.6*line_width)
plot(tNLk3,1000*sqrt(xnNLk3.^2+ynNLk3.^2),'--k','LineWidth',0.3*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('[mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$x_3$','$y_3$','$\sqrt{x_3^2 + y_3^2}$','Fontsize',legend_size,'interpreter', 'latex','Location','northeast','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

% if save_fig
%     save_name = strcat(fig_name,'_',EOMtype,'kvd=',num2str(k_),'_MSDsloshMasses_plot2','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
%     % save_name = strcat(fig_name,'_sloshMasses_plot1','.pdf');
%     % print('-dpdf', '-fillpage', save_name)
% end

