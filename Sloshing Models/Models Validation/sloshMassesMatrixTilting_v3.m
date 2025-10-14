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

path_type   = 'Tilt_RD';
dim_type    = '3D';
motion_type = [path_type,'_',dim_type];
Te          = 6.5;
A_psi       = deg2rad(30);
Dz_max      = 0.6;
Delta_start_index = 40;
sloshing_masses = [1 2 3];
motion_type_new = 'Tc3';
R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, zita1, m1, k1, c1, alphan, l1, L1, J, k, w1] = nModeParameters(R, h, 1);
[g, rho, m_tot, V, csi12, zita2, m2, k2, c2, alphan, l2, L2, J, k, w2] = nModeParameters(R, h, 2);
[g, rho, m_tot, V, csi13, zita3, m3, k3, c3, alphan, l3, L3, J, k, w3] = nModeParameters(R, h, 3);

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 1;
save_fig   = 0;

alpha_sota = [0, 0.58, 1];
w_sota = [1, 2, 1];
alpha_range = linspace(0,1,5);
w_range = [1 2 3];
alphan = 0;

w = 2;
a  = h/R;
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
fig_name = strcat('Motions_10_10_2024/Comparison/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg');
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

gammaL1 = (4*h*m1)/(rho*V*R);
gammaL2 = (4*h*m2)/(rho*V*R);
gammaL3 = (4*h*m3)/(rho*V*R);

gammaNL1 = (h*m1*csi11^2)/(rho*V*R);
gammaNL2 = (h*m2*csi12^2)/(rho*V*R);
gammaNL3 = (h*m3*csi13^2)/(rho*V*R);

time_spline = linspace(0,2*Te,2*Te*500+1);
psi_spline = linspace(0,2*pi,361);
r_spline = linspace(0,R,50);

    
[tNLt1,sNLt1] = ode45(@(t,s)odeTiltMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h1,g,alphan/R^2,w,EOMtype), tspan, S0);
etaNLt1 = gammaNL1*(sNLt1(:,1).^2 + sNLt1(:,2).^2).^0.5;
phiNLt1 = atan2(sNLt1(:,2),sNLt1(:,1));
xnNLt1 = sNLt1(:,1);
ynNLt1 = sNLt1(:,2);

[tNLt2,sNLt2] = ode45(@(t,s)odeTiltMSD(t,s,k2,k,zita2,m2,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h2,g,alphan/R^2,w,EOMtype), tspan, S0);
etaNLt2 = gammaNL2*(sNLt2(:,1).^2 + sNLt2(:,2).^2).^0.5;
phiNLt2 = atan2(sNLt2(:,2),sNLt2(:,1));
xnNLt2 = sNLt2(:,1);
ynNLt2 = sNLt2(:,2);

[tNLt3,sNLt3] = ode45(@(t,s)odeTiltMSD(t,s,k3,k,zita3,m3,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h3,g,alphan/R^2,w,EOMtype), tspan, S0);
etaNLt3 = gammaNL3*(sNLt3(:,1).^2 + sNLt3(:,2).^2).^0.5;
phiNLt3 = atan2(sNLt3(:,2),sNLt3(:,1));
xnNLt3 = sNLt3(:,1);
ynNLt3 = sNLt3(:,2);

etaNLt1_spline = spline(tNLt1,etaNLt1,time_spline);
etaNLt2_spline = spline(tNLt2,etaNLt2,time_spline);
etaNLt3_spline = spline(tNLt3,etaNLt3,time_spline);

phiNLt1_spline = spline(tNLt1,phiNLt1,time_spline);
phiNLt2_spline = spline(tNLt2,phiNLt2,time_spline);
phiNLt3_spline = spline(tNLt3,phiNLt3,time_spline);

xnNLt1_spline = spline(tNLt1,xnNLt1,time_spline);
ynNLt1_spline = spline(tNLt1,ynNLt1,time_spline);

for j = 1:length(time_spline)

    for i = 1:length(r_spline)
        etaNLt1_psi_time(:,i,j) = etaNLt1_spline(j)*besselj(1,csi11*r_spline(i)/R)/besselj(1,csi11)*cos(psi_spline - phiNLt1_spline(j));
        etaNLt2_psi_time(:,i,j) = etaNLt2_spline(j)*besselj(1,csi12*r_spline(i)/R)/besselj(1,csi12)*cos(psi_spline - phiNLt2_spline(j));
        etaNLt3_psi_time(:,i,j) = etaNLt3_spline(j)*besselj(1,csi13*r_spline(i)/R)/besselj(1,csi13)*cos(psi_spline - phiNLt3_spline(j));
    
        etaNLt1_psi_time_v2(:,i,j) = gammaNL1*besselj(1,csi11*r_spline(i)/R)/besselj(1,csi11)*(cos(psi_spline)*xnNLt1_spline(j) + sin(psi_spline)*ynNLt1_spline(j));

        end
    
        [etaNLt1_psi_time_max_rows(j,:)  , etaNLt1_max_rows(j,:)  ] = (max(etaNLt1_psi_time(:,:,j)));
        [etaNLt12_psi_time_max_rows(j,:) , etaNLt12_max_rows(j,:) ] = (max(etaNLt1_psi_time(:,:,j)+etaNLt2_psi_time(:,:,j)));
        [etaNLt123_psi_time_max_rows(j,:), etaNLt123_max_rows(j,:)] = (max(etaNLt1_psi_time(:,:,j)+etaNLt2_psi_time(:,:,j)+etaNLt3_psi_time(:,:,j)));
        [etaNLt1_psi_time_max(j) ,  etaNLt1_max_cols(j)  ] = max(etaNLt1_psi_time_max_rows(j,:));
        [etaNLt12_psi_time_max(j) , etaNLt12_max_cols(j) ] = max(etaNLt12_psi_time_max_rows(j,:));
        [etaNLt123_psi_time_max(j), etaNLt123_max_cols(j)] = max(etaNLt123_psi_time_max_rows(j,:));
        
        etaNLt1_psi_max(j)   = psi_spline(etaNLt1_max_rows(j,etaNLt1_max_cols(j)));
        etaNLt12_psi_max(j)  = psi_spline(etaNLt12_max_rows(j,etaNLt12_max_cols(j)));
        etaNLt123_psi_max(j) = psi_spline(etaNLt123_max_rows(j,etaNLt123_max_cols(j)));

        etaNLt1_psi_max_r(:,j)   = etaNLt1_psi_time(etaNLt1_max_rows(j,etaNLt1_max_cols(j)),:,j);
        etaNLt2_psi_max_r(:,j)   = etaNLt2_psi_time(etaNLt1_max_rows(j,etaNLt1_max_cols(j)),:,j);
        etaNLt3_psi_max_r(:,j)   = etaNLt3_psi_time(etaNLt1_max_rows(j,etaNLt1_max_cols(j)),:,j);
        etaNLt12_psi_max_r(:,j)  = etaNLt1_psi_time(etaNLt12_max_rows(j,etaNLt12_max_cols(j)),:,j) + etaNLt2_psi_time(etaNLt12_max_rows(j,etaNLt12_max_cols(j)),:,j);
        etaNLt123_psi_max_r(:,j) = etaNLt1_psi_time(etaNLt123_max_rows(j,etaNLt123_max_cols(j)),:,j) + etaNLt2_psi_time(etaNLt123_max_rows(j,etaNLt123_max_cols(j)),:,j) + etaNLt3_psi_time(etaNLt123_max_rows(j,etaNLt123_max_cols(j)),:,j);

        [etaNLt1_psi_time_maxR(j), etaNLt1_ind_maxR(j)] = (max(etaNLt1_psi_time(:,end,j)));
        [etaNLt12_psi_time_maxR(j), etaNLt12_ind_maxR(j)] = (max(etaNLt1_psi_time(:,end,j)+etaNLt2_psi_time(:,end,j)));
        [etaNLt123_psi_time_maxR(j), etaNLt123_ind_maxR(j)] = (max(etaNLt1_psi_time(:,end,j)+etaNLt2_psi_time(:,end,j)+etaNLt3_psi_time(:,end,j)));

end

[max_eta, index_eta] = max(etaNLt1_spline);
% index_eta = 713;
etaNLt2_r = etaNLt2_psi_time(etaNLt123_max_rows(index_eta,etaNLt123_max_cols(index_eta)),:,index_eta);
etaNLt3_r = etaNLt3_psi_time(etaNLt123_max_rows(index_eta,etaNLt123_max_cols(index_eta)),:,index_eta);

etaNLt2_phi1max_r = etaNLt2_psi_max_r(:,index_eta);
etaNLt3_phi1max_r = etaNLt3_psi_max_r(:,index_eta);

etaNLt1_r  = etaNLt1_psi_max_r(:,index_eta);
etaNLt12_r  = etaNLt12_psi_max_r(:,index_eta);
etaNLt123_r = etaNLt123_psi_max_r(:,index_eta);

%% P-tan
[tPNLt1,sPNLt1] = ode45(@(t,s)odeTiltP(t,s,l1,L1,zita1,m1,time,rEddy,-rEddx,rEddz,psiOde,psidOde,psiddOde,h,h1,g,alphan,w1,EOMtype),tspan,S0);
phixPNLt1 = sPNLt1(:,2);
phiyPNLt1 = sPNLt1(:,1);
xnPNLt1 = l1*sin(phiyPNLt1).*cos(phixPNLt1);
ynPNLt1 = l1*sin(phixPNLt1);
phiPNLt1 = atan2(ynPNLt1,xnPNLt1);

for j=1:length(tPNLt1)
     etaPNLt1_tan(j) = getSloshHeight(sPNLt1(j,1),sPNLt1(j,2),R);
end

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

%% Accuracy-index computation
eps_m1 = calcola_errore_picco(1000*etaNLt1_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_m2 = calcola_errore_picco(1000*etaNLt12_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_m3  = calcola_errore_picco(1000*etaNLt123_psi_time_max, abs(E_slosh_h_cut_1), 0, 1);
eps_n1 = calcola_errore_picco(1000*(etaNLt1_spline), abs(E_slosh_h_cut_1), 0, 1);
eps_n2 = calcola_errore_picco(1000*(etaNLt1_spline+etaNLt2_spline), abs(E_slosh_h_cut_1), 0, 1);
eps_n3  = calcola_errore_picco(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline), abs(E_slosh_h_cut_1), 0, 1);

[eps_m1, eps_m2, eps_m3; eps_n1, eps_n2, eps_n3]

sigma_m1 = media_errore(1000*etaNLt1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_m2 = media_errore(1000*etaNLt12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_m3 = media_errore(1000*etaNLt123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n1_old = media_errore(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n2_old = media_errore(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigma_n3_old = media_errore(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

sigma_n1 = errore_integrale(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n2 = errore_integrale(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n3 = errore_integrale(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,0);
sigma_n1_norm = errore_integrale(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);
sigma_n2_norm = errore_integrale(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);
sigma_n3_norm = errore_integrale(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1);

[sigma_m1, sigma_m2, sigma_m3; sigma_n1, sigma_n2, sigma_n3]

epsR_m1 = calcola_errore_picco_esaurimento(1000*etaNLt1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_m2 = calcola_errore_picco_esaurimento(1000*etaNLt12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_m3 = calcola_errore_picco_esaurimento(1000*etaNLt123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n1 = calcola_errore_picco_esaurimento(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n2 = calcola_errore_picco_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
epsR_n3 = calcola_errore_picco_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

[epsR_m1, epsR_m2, epsR_m3; epsR_n1, epsR_n2, epsR_n3]

sigmaR_m1 = media_errore_esaurimento(1000*etaNLt1_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_m2 = media_errore_esaurimento(1000*etaNLt12_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_m3 = media_errore_esaurimento(1000*etaNLt123_psi_time_max,abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n1_old = media_errore_esaurimento(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n2_old = media_errore_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);
sigmaR_n3_old = media_errore_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te);

sigmaR_n1 = errore_integrale_esaurimento(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n2 = errore_integrale_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n3 = errore_integrale_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,0);
sigmaR_n1_norm = errore_integrale_esaurimento(1000*(etaNLt1_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);
sigmaR_n2_norm = errore_integrale_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);
sigmaR_n3_norm = errore_integrale_esaurimento(1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),abs(E_slosh_h_cut_1),time_spline,E_slosh_t_cut_1,Te,1.5*Te,1);

[sigmaR_m1, sigmaR_m2, sigmaR_m3; sigmaR_n1, sigmaR_n2, sigmaR_n3]

%% Graphics
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
title(motion_type_new,'interpreter', 'latex')
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim-10])
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.7*line_width)
plot(time_spline,1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),'--','Color',"#77AC30",'LineWidth',0.7*line_width)
plot(tPNLt1,1000*etaPNLt1_tan,':','Color','#7E2F8E','LineWidth',0.7*line_width)
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
plot(time_spline,rad2deg(etaNLt1_psi_max),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(etaNLt12_psi_max),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(etaNLt123_psi_max),'LineWidth',0.6*line_width)
% plot(time_spline,rad2deg(etaNLt123_psi_max),'--k')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_{max}$ [deg]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
grid on
box on 
plot(1000*r_spline,1000*etaNLt1_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*etaNLt12_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*etaNLt123_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt2_r),'--','Color',"#D95319",'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt3_r),'--','Color',"#EDB120",'LineWidth',0.6*line_width)
xlabel('$r$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('${\eta}(\phi^*,r,t^*)$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,4)
hold on
grid on
box on 
xlim([0 1.5*Te])
ylim([0 Ylim])
plot(time_spline,1000*etaNLt1_spline,'LineWidth',0.6*line_width)
plot(time_spline,1000*(etaNLt1_spline+etaNLt2_spline),'LineWidth',0.6*line_width)
plot(time_spline,1000*(etaNLt1_spline+etaNLt2_spline+etaNLt3_spline),'LineWidth',0.6*line_width)
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),':','LineWidth',0.6*line_width,'Color','#7E2F8E')
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\sum_{k=1}^n\overline{\eta}_k$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$n=1$','$n=2$','$n=3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,5)
hold on
grid on
box on 
xlim([0 1.0*Te])
plot(time_spline,rad2deg(wrapTo2Pi(phiNLt1_spline)),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(wrapTo2Pi(phiNLt2_spline)),'LineWidth',0.6*line_width)
plot(time_spline,rad2deg(wrapTo2Pi(phiNLt3_spline)),'LineWidth',0.6*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\phi_n$ [deg]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$n=1$','$n=2$','$n=3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,6)
hold on
grid on
box on 
plot(1000*r_spline,1000*etaNLt1_r,'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt1_r+etaNLt2_phi1max_r),'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt1_r+etaNLt2_phi1max_r+etaNLt3_phi1max_r),'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt2_phi1max_r),'--','Color',"#D95319",'LineWidth',0.6*line_width)
plot(1000*r_spline,1000*(etaNLt3_phi1max_r),'--','Color',"#EDB120",'LineWidth',0.6*line_width)
xlabel('$r$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('${\eta}(\phi^*,r,t^*)$ [mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$m_1$','$m_1+m_2$','$m_1+m_2+m_3$','Fontsize',legend_size,'interpreter', 'latex','Location','best','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}-Bessel}$'],'interpreter', 'latex')
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
    etaNL_total = 1000 * (etaNLt1_spline(:) + etaNLt2_spline(:) + etaNLt3_spline(:));
    T2 = table(time_spline(:), etaNL_total, ...
        'VariableNames', {'Time', 'Eta_PMD'});
    writetable(T2, fullfile('csv', strcat(motion_type_new, '_PMD.csv')));
    
    % Third dataset
    T3 = table(tPNLt1(:), 1000 * etaPNLt1_tan(:), ...
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
        '$\sigma_r$ [mm]', sigmaR_n1, sigmaR_n2, sigmaR_n3; 
        '$\overline{\sigma}$ [\%]',   sigma_n1_norm, sigma_n2_norm, sigma_n3_norm;
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
plot(tNLt1,1000*xnNLt1,'LineWidth',0.6*line_width)
plot(tNLt1,1000*ynNLt1,'LineWidth',0.6*line_width)
plot(tNLt1,1000*sqrt(xnNLt1.^2+ynNLt1.^2),'--k','LineWidth',0.3*line_width)
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
plot(tNLt2,1000*xnNLt2,'LineWidth',0.6*line_width)
plot(tNLt2,1000*ynNLt2,'LineWidth',0.6*line_width)
plot(tNLt2,1000*sqrt(xnNLt2.^2+ynNLt2.^2),'--k','LineWidth',0.3*line_width)
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
plot(tNLt3,1000*xnNLt3,'LineWidth',0.6*line_width)
plot(tNLt3,1000*ynNLt3,'LineWidth',0.6*line_width)
plot(tNLt3,1000*sqrt(xnNLt3.^2+ynNLt3.^2),'--k','LineWidth',0.3*line_width)
xlabel('$t$ [s]','FontSize', label_size, 'Interpreter', 'latex');
ylabel('[mm]','FontSize', label_size, 'Interpreter', 'latex');
leg = legend('$x_3$','$y_3$','$\sqrt{x_3^2 + y_3^2}$','Fontsize',legend_size,'interpreter', 'latex','Location','northeast','NumColumns',1);
title(leg,['$\mathrm{',EOMtype,'_{MSD}}$'],'interpreter', 'latex')
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
