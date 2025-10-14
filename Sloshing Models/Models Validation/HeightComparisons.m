%--------------------------------------------------------------------------
% Pendulum Height formulas Comparison for MATRIX project
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it 
%    Date:       February 2025
%--------------------------------------------------------------------------
clear all
close all
clc

% addpath('C:\Users\Utente\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5');
% addpath("C:\Users\simon\Desktop\Matrix_Project_git\Sliding_Optimization\casadi-3.6.5-windows64-matlab2018b\")
% addpath('Comau_Kinematics');
% addpath('Pendulum');
% addpath('utils\');
% addpath('ode\');

ws_path = fullfile('..', '..', '..');
kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
odes_path = fullfile("odes/");

% add folder and subfolders
addpath(genpath(kin_path));
addpath(genpath(odes_path));

addpath(genpath('utils\'));
addpath(genpath('height_comparisons\'));

%%
path_type   = 'LE';
dim_type    = '2D';
motion_type = [path_type,'_',dim_type];
Te          = 4.8;
Dz_max      = 0.6;
Delta_start_index = 28;
% Delta_start_index = 0;


R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, Cs, ms, ks, cs, as, ls, Ls, J, k, wn] = ParametersP(R, h);
% Cs = Cs*1.5;
% R = 0.035
% Cs = 0.028571;
wn = 18.961156;

preon_flag = 0; % flag 0=exp, 1=preon
save_video = 0;
save_csv   = 0;
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
fig_name = strcat('Motions_10_10_2024/Post_processing/Pendulum/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg');
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

%% Sloshing-Height Formulation (Parabola)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with paraboloic term 
[tNLp,sNL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,2,'NL'), tspan, S0);
gammaNL = (h*ms*csi11^2)/(rho*V*R);
etaNL = gammaNL*(sNL(:,1).^2 + sNL(:,2).^2).^0.5;

for i = 1:length(tNLp)
    if tNLp(i)<Te
        thetad(i) = spline(time,thd,tNLp(i));
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

%%L with paraboloic term 
[tLp,sL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,2,'L'), tspan, S0);
gammaL = (4*h*ms)/(rho*V*R);
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

% Sloshing-Height Formulation (Viscosity)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL with viscosity 
[tNLk,sNLk] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd,J,g,as,2,'NL'), tspan, S0);
gammaNL = (h*ms*csi11^2)/(rho*V*R);
etaNLk = gammaNL*(sNLk(:,1).^2 + sNLk(:,2).^2).^0.5;

%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L with viscosity
[tLk,sLk] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd,J,g,as,2,'L'), tspan, S0);
gammaL = (4*h*ms)/(rho*V*R);
etaLk = gammaL*(sLk(:,1).^2 + sLk(:,2).^2).^0.5;
%% Sloshing-Height Formulation (Only translation)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL 
[tNL,sNL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,2,'NL'), tspan, S0);
gammaNL = (h*ms*csi11^2)/(rho*V*R);
etaNL_MSD = gammaNL*(sNL(:,1).^2 + sNL(:,2).^2).^0.5;

%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L  
[tL,sL] = ode45(@(t,s)odeSchoenMSD(t,s,ks,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,2,'L'), tspan, S0);
gammaL = (4*h*ms)/(rho*V*R);
etaL_MSD = gammaL*(sL(:,1).^2 + sL(:,2).^2).^0.5;

%% Sloshing-Height Formulation Pendulum (Only translation)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL 
[tPNL,sPNL] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'NL'), tspan, S0);

etaPNL = zeros(length(tPNL),1);
for i=1:length(tPNL)
    % etaPNL(i) = getSloshHeight(sPNL(i,1),sPNL(i,2),R);
    etaPNL(i) = getSloshHeight2(sPNL(i,1),sPNL(i,2),R);
end


%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L  
[tPL,sPL] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'L'), tspan, S0);


etaPL = zeros(length(tPL),1);
for i=1:length(tPL)
    % etaPL(i) = getSloshHeight(sPL(i,1),sPL(i,2),R);
    etaPL(i) = getSloshHeight2(sPL(i,1),sPL(i,2),R);
end

%% Sloshing-Height Formulation Pendulum (BESSEL HEIGHT)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL 
[tPNL_NL,sPNL_NL] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'NL'), tspan, S0);

etaPNL_NL = zeros(length(tPNL_NL),1);
gammaPNL_NL = ((h*ms)/(rho*V)) * (1/tanh(csi11*h/R));
for i=1:length(tPNL_NL)
    % etaPNL_NL(i) = gammaPNL_NL*(2*(csi11^2 - 1)*(1-cos(sPNL_NL(i,1))*cos(sPNL_NL(i,2)))).^0.5;
    etaPNL_NL(i) = gammaPNL_NL*csi11*(2*(1-cos(sPNL_NL(i,1))*cos(sPNL_NL(i,2))))^0.5;
end

%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L  
[tPL_NL,sPL_NL] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'NL'), tspan, S0);

etaPL_NL = zeros(length(tPL_NL),1);
gammaPL_NL = (h*ms)/(rho*V) * 1/tanh(csi11*h/R);
for i=1:length(tPL_NL)
    % etaPL_NL(i) = gammaPL_NL*(2*(csi11^2 - 1)*(1-cos(sPL_NL(i,1))*cos(sPL_NL(i,2)))).^0.5;
    % etaPL_NL(i) = gammaPL_NL*csi11*(2*(1-cos(sPL_NL(i,1))*cos(sPL_NL(i,2))))^0.5;
    etaPL_NL(i) = gammaPL_NL*csi11*(1-cos(sPL_NL(i,1))^2*cos(sPL_NL(i,2))^2)^0.5;
end

%% Sloshing-Height Formulation Pendulum (PLANAR HEIGHT)
%%Non-linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%NL 
[tPNL_L,sPNL_L] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'NL'), tspan, S0);

etaPNL_L = zeros(length(tPNL_L),1);
gammaPNL_L = (h*ms)/(rho*V) * 1/tanh(csi11*h/R);
for i=1:length(tPNL_L)
    % etaPNL_L(i) = gammaPNL_L*csi11*(2*(1-cos(sPNL_L(i,1))*cos(sPNL_L(i,2)))).^0.5;
    etaPNL_L(i) = gammaPNL_L*2*((csi11^2 - 1)*(1-cos(sPNL_L(i,1))*cos(sPNL_L(i,2))))^0.5;
end

%%Linear Sloshing Model
tspan = [0 2*Te];
S0 = [0 0 0 0 0 0 0];
thdd_zero = zeros(1,n);

%%L  
[tPL_L,sPL_L] = ode45(@(t,s)odeSchoenP(t,s,ls,k,Cs,ms,time,rEddy,-rEddx,rEddz,thdd_zero,J,g,as,wn,'NL'), tspan, S0);


etaPL_L = zeros(length(tPL_L),1);
gammaPL_L = (h*ms)/(rho*V) * 1/tanh(csi11*h/R);
for i=1:length(tPL_L)
% etaPL_L(i) = gammaPL_L*csi11*(2*(1-cos(sPL_L(i,1))*cos(sPL_L(i,2)))).^0.5;
% etaPL_L(i) = gammaPL_L*2*((csi11^2 - 1)*(1-cos(sPL_L(i,1))*cos(sPL_L(i,2))))^0.5;
etaPL_L(i) = gammaPL_L*4/csi11 *(1-cos(sPL_L(i,1))^2*cos(sPL_L(i,2))^2)^0.5;
end

%% Phasing procedure
Efps = 60; % Framerate experiment video
end_time = 1.5*Te;
[~, emax_index] = max(slosh_heights);
[~, Mmax_index] = max(etaLPar);
peak_time = tLp(Mmax_index);

[~, MPmax_index] = max(etaPL);
Ppeak_time = tPL(MPmax_index);

E_start_index = round(emax_index - peak_time*Efps) + Delta_start_index;
E_slosh_h_cut_1 = slosh_heights(E_start_index+1:E_start_index+(end_time*Efps));
E_slosh_t_cut_1 = slosh_times(E_start_index+1:E_start_index+(end_time*Efps)) - slosh_times(E_start_index+1);

E_pstart_index = round(emax_index - Ppeak_time*Efps) + Delta_start_index;
E_pslosh_h_cut_1 = slosh_heights(E_pstart_index+1:E_pstart_index+(end_time*Efps));
E_pslosh_t_cut_1 = slosh_times(E_pstart_index+1:E_pstart_index+(end_time*Efps)) - slosh_times(E_pstart_index+1);

%% Accuracy-index computation
eps_NLp = calcola_errore_picco(etaNLPar*1000, abs(E_slosh_h_cut_1), 0, 1);
eps_NLk = calcola_errore_picco(etaNLk*1000, abs(E_slosh_h_cut_1), 0, 1);
eps_NL  = calcola_errore_picco(etaNL_MSD*1000, abs(E_slosh_h_cut_1), 0, 1);
eps_Lp = calcola_errore_picco(etaLPar*1000, abs(E_slosh_h_cut_1), 0, 1);
eps_Lk = calcola_errore_picco(etaLk*1000, abs(E_slosh_h_cut_1), 0, 1);
eps_L  = calcola_errore_picco(etaL_MSD*1000, abs(E_slosh_h_cut_1), 0, 1);

sigma_NLp = media_errore(etaNLPar*1000,abs(E_slosh_h_cut_1),tNLp,E_slosh_t_cut_1,Te);
sigma_NLk = media_errore(etaNLk*1000,abs(E_slosh_h_cut_1),tNLk,E_slosh_t_cut_1,Te);
sigma_NL  = media_errore(etaNL_MSD*1000,abs(E_slosh_h_cut_1),tNL,E_slosh_t_cut_1,Te);
sigma_Lp = media_errore(etaLPar*1000,abs(E_slosh_h_cut_1),tLp,E_slosh_t_cut_1,Te);
sigma_Lk = media_errore(etaLk*1000,abs(E_slosh_h_cut_1),tLk,E_slosh_t_cut_1,Te);
sigma_L  = media_errore(etaL_MSD*1000,abs(E_slosh_h_cut_1),tL,E_slosh_t_cut_1,Te);

epsR_NLp = calcola_errore_picco_esaurimento(etaNLPar*1000,abs(E_slosh_h_cut_1),tNLp,E_slosh_t_cut_1,Te);
epsR_NLk = calcola_errore_picco_esaurimento(etaNLk*1000,abs(E_slosh_h_cut_1),tNLk,E_slosh_t_cut_1,Te);
epsR_NL  = calcola_errore_picco_esaurimento(etaNL_MSD*1000,abs(E_slosh_h_cut_1),tNL,E_slosh_t_cut_1,Te);
epsR_Lp = calcola_errore_picco_esaurimento(etaLPar*1000,abs(E_slosh_h_cut_1),tLp,E_slosh_t_cut_1,Te);
epsR_Lk = calcola_errore_picco_esaurimento(etaLk*1000,abs(E_slosh_h_cut_1),tLk,E_slosh_t_cut_1,Te);
epsR_L  = calcola_errore_picco_esaurimento(etaL_MSD*1000,abs(E_slosh_h_cut_1),tL,E_slosh_t_cut_1,Te);

sigmaR_NLp = media_errore_esaurimento(etaNLPar*1000,abs(E_slosh_h_cut_1),tNLp,E_slosh_t_cut_1,Te);
sigmaR_NLk = media_errore_esaurimento(etaNLk*1000,abs(E_slosh_h_cut_1),tNLk,E_slosh_t_cut_1,Te);
sigmaR_NL  = media_errore_esaurimento(etaNL_MSD*1000,abs(E_slosh_h_cut_1),tNL,E_slosh_t_cut_1,Te);
sigmaR_Lp = media_errore_esaurimento(etaLPar*1000,abs(E_slosh_h_cut_1),tLp,E_slosh_t_cut_1,Te);
sigmaR_Lk = media_errore_esaurimento(etaLk*1000,abs(E_slosh_h_cut_1),tLk,E_slosh_t_cut_1,Te);
sigmaR_L  = media_errore_esaurimento(etaL_MSD*1000,abs(E_slosh_h_cut_1),tL,E_slosh_t_cut_1,Te);


%% Pendulum
% eps_PNLp = calcola_errore_picco(etaPNLPar*1000, abs(E_pslosh_h_cut_1), 0, 1);
% eps_PNLk = calcola_errore_picco(etaPNLk*1000, abs(E_pslosh_h_cut_1), 0, 1);
eps_PNL  = calcola_errore_picco(etaPNL*1000, abs(E_pslosh_h_cut_1), 0, 1);
% eps_PLp = calcola_errore_picco(etaPLPar*1000, abs(E_pslosh_h_cut_1), 0, 1);
% eps_PLk = calcola_errore_picco(etaPLk*1000, abs(E_pslosh_h_cut_1), 0, 1);
eps_PL  = calcola_errore_picco(etaPL*1000, abs(E_pslosh_h_cut_1), 0, 1);

eps_PNL_NL  = calcola_errore_picco(etaPNL_NL*1000, abs(E_pslosh_h_cut_1), 0, 1);
eps_PNL_L  = calcola_errore_picco(etaPNL_L*1000, abs(E_pslosh_h_cut_1), 0, 1);
eps_PL_NL  = calcola_errore_picco(etaPL_NL*1000, abs(E_pslosh_h_cut_1), 0, 1);
eps_PL_L  = calcola_errore_picco(etaPL_L*1000, abs(E_pslosh_h_cut_1), 0, 1);



% sigma_PNLp = media_errore(etaPNLPar*1000,abs(E_pslosh_h_cut_1),tPNLp,E_pslosh_t_cut_1,Te);
% sigma_PNLk = media_errore(etaPNLk*1000,abs(E_pslosh_h_cut_1),tPNLk,E_pslosh_t_cut_1,Te);
sigma_PNL  = media_errore(etaPNL*1000,abs(E_pslosh_h_cut_1),tPNL,E_pslosh_t_cut_1,Te);
% sigma_PLp = media_errore(etaPLPar*1000,abs(E_pslosh_h_cut_1),tPLp,E_pslosh_t_cut_1,Te);
% sigma_PLk = media_errore(etaPLk*1000,abs(E_pslosh_h_cut_1),tPLk,E_pslosh_t_cut_1,Te);
sigma_PL  = media_errore(etaPL*1000,abs(E_pslosh_h_cut_1),tPL,E_pslosh_t_cut_1,Te);

sigma_PNL_NL  = media_errore(etaPNL_NL*1000,abs(E_pslosh_h_cut_1),tPNL_NL,E_pslosh_t_cut_1,Te);
sigma_PL_NL  = media_errore(etaPL_NL*1000,abs(E_pslosh_h_cut_1),tPL_NL,E_pslosh_t_cut_1,Te);
sigma_PNL_L  = media_errore(etaPNL_L*1000,abs(E_pslosh_h_cut_1),tPNL_L,E_pslosh_t_cut_1,Te);
sigma_PL_L  = media_errore(etaPL_L*1000,abs(E_pslosh_h_cut_1),tPL_L,E_pslosh_t_cut_1,Te);


% epsR_PNLp = calcola_errore_picco_esaurimento(etaPNLPar*1000,abs(E_pslosh_h_cut_1),tPNLp,E_pslosh_t_cut_1,Te);
% epsR_PNLk = calcola_errore_picco_esaurimento(etaPNLk*1000,abs(E_pslosh_h_cut_1),tPNLk,E_pslosh_t_cut_1,Te);
epsR_PNL  = calcola_errore_picco_esaurimento(etaPNL*1000,abs(E_pslosh_h_cut_1),tPNL,E_pslosh_t_cut_1,Te);
% epsR_PLp = calcola_errore_picco_esaurimento(etaPLPar*1000,abs(E_pslosh_h_cut_1),tPLp,E_pslosh_t_cut_1,Te);
% epsR_PLk = calcola_errore_picco_esaurimento(etaPLk*1000,abs(E_pslosh_h_cut_1),tPLk,E_pslosh_t_cut_1,Te);
epsR_PL  = calcola_errore_picco_esaurimento(etaPL*1000,abs(E_pslosh_h_cut_1),tPL,E_pslosh_t_cut_1,Te);

epsR_PNL_NL  = calcola_errore_picco_esaurimento(etaPNL_NL*1000,abs(E_pslosh_h_cut_1),tPNL_NL,E_pslosh_t_cut_1,Te);
epsR_PL_NL  = calcola_errore_picco_esaurimento(etaPL_NL*1000,abs(E_pslosh_h_cut_1),tPL_NL,E_pslosh_t_cut_1,Te);
epsR_PNL_L  = calcola_errore_picco_esaurimento(etaPNL_L*1000,abs(E_pslosh_h_cut_1),tPNL_L,E_pslosh_t_cut_1,Te);
epsR_PL_L  = calcola_errore_picco_esaurimento(etaPL_L*1000,abs(E_pslosh_h_cut_1),tPL_L,E_pslosh_t_cut_1,Te);



% sigmaR_PNLp = media_errore_esaurimento(etaPNLPar*1000,abs(E_pslosh_h_cut_1),tPNLp,E_pslosh_t_cut_1,Te);
% sigmaR_PNLk = media_errore_esaurimento(etaPNLk*1000,abs(E_pslosh_h_cut_1),tPNLk,E_pslosh_t_cut_1,Te);
sigmaR_PNL  = media_errore_esaurimento(etaPNL*1000,abs(E_pslosh_h_cut_1),tPNL,E_pslosh_t_cut_1,Te);
% sigmaR_PLp = media_errore_esaurimento(etaPLPar*1000,abs(E_pslosh_h_cut_1),tPLp,E_pslosh_t_cut_1,Te);
% sigmaR_PLk = media_errore_esaurimento(etaPLk*1000,abs(E_pslosh_h_cut_1),tPLk,E_pslosh_t_cut_1,Te);
sigmaR_PL  = media_errore_esaurimento(etaPL*1000,abs(E_pslosh_h_cut_1),tPL,E_pslosh_t_cut_1,Te);


sigmaR_PNL_NL  = media_errore_esaurimento(etaPNL_NL*1000,abs(E_pslosh_h_cut_1),tPNL_NL,E_pslosh_t_cut_1,Te);
sigmaR_PL_NL  = media_errore_esaurimento(etaPL_NL*1000,abs(E_pslosh_h_cut_1),tPL_NL,E_pslosh_t_cut_1,Te);
sigmaR_PNL_L  = media_errore_esaurimento(etaPNL_L*1000,abs(E_pslosh_h_cut_1),tPNL_L,E_pslosh_t_cut_1,Te);
sigmaR_PL_L  = media_errore_esaurimento(etaPL_L*1000,abs(E_pslosh_h_cut_1),tPL_L,E_pslosh_t_cut_1,Te);


%% Graphics
label_size  = 14;
axis_size   = 14;
legend_size = 14;
line_width  = 2.5;
num_cols    = 3;

max_slosh_mod = max(max(etaPL*1000),max(etaL*1000));
max_slosh = max(max_slosh_mod,max(E_slosh_h_cut_1));
Ylim      = (floor(max_slosh/5)+1)*5;

% figure()
% hold on
% grid on
% box on 
% plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
% plot(tL, etaLPar*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
% plot(tNL, etaNLPar*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
% line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
% xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
% xlim([0 1.5*Te])
% ylim([0 Ylim])
% legend('Experimentt','L Model', 'NL Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% if save_fig
%     save_name = strcat(fig_name,'.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% %     save_name = strcat(fig_name,'.pdf');
% %     print('-dpdf', '-fillpage', save_name)
% end

fig = figure()
title("MSD")
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
plot(time,th,'LineWidth',line_width)
plot(time,thd,'LineWidth',line_width)
plot(time,thdd,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\theta_0$ [rad]','$\dot \theta_0$ [rad/s]', '$\ddot \theta_0$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
box off
axis off
xlim([0 5])
ylim([0 5])
label_size = 11;
text(-1,5,['$||\ddot{\mathbf{r}}||_{max} = $',num2str(max(acc_norm)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4.5,['$||\ddot{\mathbf{r}}_{2D}||_{max} = $',num2str(max(acc_norm2D)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4,['$|\ddot{r}_z|_{max} = $',num2str(max((abs(rEddz)))),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.5,['$|\ddot{\theta}|_{max} = $',num2str(max((abs(thdd)))),'[rad/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.0,['$\overline \eta_{L_p,max} = $',num2str(max(etaLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.5,['$\overline \eta_{NL_p,max} = $',num2str(max(etaNLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.0,['$\overline \eta_{max} = $',num2str(max(abs(E_slosh_h_cut_1))),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,1.5,['$T_{e} = $',num2str(Te),'[s]'],'FontSize', label_size, 'Interpreter', 'latex')
text(2,5.5,type_name,'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,5,['$\epsilon_{\%,NL_p} = ',num2str(eps_NLp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,4.5,['$\epsilon_{\%,NL_k} = ',num2str(eps_NLk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,4,['$\epsilon_{\%,NL} = ',num2str(eps_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,3.5,['$\epsilon_{\%,L_p} = ',num2str(eps_Lp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,3,['$\epsilon_{\%,L_k} = ',num2str(eps_Lk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,2.5,['$\epsilon_{\%,L} = ',num2str(eps_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,5,['$\epsilon_{r,\%,NL_p} = ',num2str(epsR_NLp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,4.5,['$\epsilon_{r,\%,NL_k} = ',num2str(epsR_NLk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,4,['$\epsilon_{r,\%,NL} = ',num2str(epsR_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,3.5,['$\epsilon_{r,\%,L_p} = ',num2str(epsR_Lp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,3,['$\epsilon_{r,\%,L_k} = ',num2str(epsR_Lk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,2.5,['$\epsilon_{r,\%,L} = ',num2str(epsR_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,2,['$\sigma_{\%,NL_p} = ',num2str(sigma_NLp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,1.5,['$\sigma_{\%,NL_k} = ',num2str(sigma_NLk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,1,['$\sigma_{\%,NL} = ',num2str(sigma_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,0.5,['$\sigma_{\%,L_p} = ',num2str(sigma_Lp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,0,['$\sigma_{\%,L_k} = ',num2str(sigma_Lk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,-0.5,['$\sigma_{\%,L} = ',num2str(sigma_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,2,['$\sigma_{r,\%,NL_p} = ',num2str(sigmaR_NLp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,1.5,['$\sigma_{r,\%,NL_k} = ',num2str(sigmaR_NLk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,1,['$\sigma_{r,\%,NL} = ',num2str(sigmaR_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,0.5,['$\sigma_{r,\%,L_p} = ',num2str(sigmaR_Lp),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,0,['$\sigma_{r,\%,L_k} = ',num2str(sigmaR_Lk),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,-0.5,['$\sigma_{r,\%,L} = ',num2str(sigmaR_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
label_size = 14;
subplot(2,3,4)
hold on
grid on
box on 
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tLp, etaLPar*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
plot(tNLp, etaNLPar*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
legend('Experiment','$\mathrm{L_p}$ Model', '$\mathrm{NL_p}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,5)
hold on
grid on
box on 
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tLk, etaLk*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
plot(tNLk, etaNLk*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
legend('Experiment','$\mathrm{L_k}$ Model', '$\mathrm{NL_k}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,6)
hold on
grid on
box on 
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tL, etaL_MSD*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#77AC30');
plot(tNL, etaNL_MSD*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
legend('Experiment','$\mathrm{L}$ Model', '$\mathrm{NL}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')


if save_fig
    save_name = strcat(fig_name,'.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'.pdf');
    print('-dpdf', '-fillpage', save_name)
end

%% pendulum
fig2 = figure()
title("PENDULUM")
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig2,'Position');
set(fig2,'Units','Normalized');
set(fig2,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
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
plot(time,th,'LineWidth',line_width)
plot(time,thd,'LineWidth',line_width)
plot(time,thdd,'LineWidth',line_width)
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 Te])
legend('$\theta_0$ [rad]','$\dot \theta_0$ [rad/s]', '$\ddot \theta_0$ [rad/s$^2$]','Location','best','Fontsize',legend_size,'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
subplot(2,3,3)
hold on
box off
axis off
xlim([0 5])
ylim([0 5])
label_size = 11;
text(-1,5,['$||\ddot{\mathbf{r}}||_{max} = $',num2str(max(acc_norm)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4.5,['$||\ddot{\mathbf{r}}_{2D}||_{max} = $',num2str(max(acc_norm2D)),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,4,['$|\ddot{r}_z|_{max} = $',num2str(max((abs(rEddz)))),'[m/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.5,['$|\ddot{\theta}|_{max} = $',num2str(max((abs(thdd)))),'[rad/s$^2$]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,3.0,['$\overline \eta_{L_p,max} = $',num2str(max(etaLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.5,['$\overline \eta_{NL_p,max} = $',num2str(max(etaNLPar*1000)),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,2.0,['$\overline \eta_{max} = $',num2str(max(abs(E_slosh_h_cut_1))),'[mm]'],'FontSize', label_size, 'Interpreter', 'latex')
text(-1,1.5,['$T_{e} = $',num2str(Te),'[s]'],'FontSize', label_size, 'Interpreter', 'latex')
text(2,5.5,type_name,'FontSize', label_size, 'Interpreter', 'latex')

text(1.5,5,['$\epsilon_{\%,NL_NL} = ',num2str(eps_PNL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,4.5,['$\epsilon_{\%,NL_L} = ',num2str(eps_PNL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,4,['$\epsilon_{\%,NL} = ',num2str(eps_PNL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(1.5,3.5,['$\epsilon_{\%,L_NL} = ',num2str(eps_PL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,3,['$\epsilon_{\%,L_L} = ',num2str(eps_PL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,2.5,['$\epsilon_{\%,L} = ',num2str(eps_PL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(3.5,5,['$\epsilon_{r,\%,NL_NL} = ',num2str(epsR_PNL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,4.5,['$\epsilon_{r,\%,NL_L} = ',num2str(epsR_PNL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,4,['$\epsilon_{r,\%,NL} = ',num2str(epsR_PNL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(3.5,3.5,['$\epsilon_{r,\%,L_NL} = ',num2str(epsR_PL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,3,['$\epsilon_{r,\%,L_NL} = ',num2str(epsR_PL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,2.5,['$\epsilon_{r,\%,L} = ',num2str(epsR_PL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(1.5,2,['$\sigma_{\%,NL_NL} = ',num2str(sigma_PNL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,1.5,['$\sigma_{\%,NL_L} = ',num2str(sigma_PNL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,1,['$\sigma_{\%,NL} = ',num2str(sigma_PNL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(1.5,0.5,['$\sigma_{\%,L_p} = ',num2str(sigma_PL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,0,['$\sigma_{\%,L_k} = ',num2str(sigma_PL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(1.5,-0.5,['$\sigma_{\%,L} = ',num2str(sigma_PL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(3.5,2,['$\sigma_{r,\%,NL_p} = ',num2str(sigmaR_PNL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,1.5,['$\sigma_{r,\%,NL_k} = ',num2str(sigmaR_PNL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,1,['$\sigma_{r,\%,NL} = ',num2str(sigmaR_PNL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')

text(3.5,0.5,['$\sigma_{r,\%,L_p} = ',num2str(sigmaR_PL_NL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,0,['$\sigma_{r,\%,L_k} = ',num2str(sigmaR_PL_L),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
text(3.5,-0.5,['$\sigma_{r,\%,L} = ',num2str(sigmaR_PL),' \, \mathrm{\%}$'],'FontSize', label_size, 'Interpreter', 'latex')
label_size = 14;

subplot(2,3,4)
hold on
grid on
box on 

plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tPL_NL, etaPL_NL*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL_NL, etaPNL_NL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#942622');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
lg = legend('Experiment','$\mathrm{PL_{bessel}}$ Model', '$\mathrm{PNL_{bessel}}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
lg.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg.Position; % [x, y, width, height]
lg.Position(2) = currentPosition(2) + 0.09;
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,5)
hold on
grid on
box on 
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tPL_L, etaPL_L*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL_L, etaPNL_L*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#942622');
line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
lg2 = legend('Experiment','$\mathrm{PL_{planar}}$ Model', '$\mathrm{PNL_{planar}}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
lg2.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg2.Position; % [x, y, width, height]
lg2.Position(2) = currentPosition(2) + 0.09;
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');

subplot(2,3,6)
hold on
grid on
box on 
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tPL, etaPL*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL, etaPNL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#942622');
% plot(tL, etaL*1000,'LineStyle','--','LineWidth',0.8*line_width,'Color','red');
% plot(tNL, etaNL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','red');


line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'FontSize', label_size, 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]', 'FontSize', label_size, 'Interpreter', 'latex');
xlim([0 1.5*Te])
ylim([0 Ylim])
lg3 = legend('Experiment','$\mathrm{PL}$ Model', '$\mathrm{PNL}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')
lg3.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg3.Position; % [x, y, width, height]
lg3.Position(2) = currentPosition(2) + 0.09;


%%
fig_name = "height_comparison\";
fig_name = strcat(fig_name, path_type,"\");
save_fig = 1;


figure()
hold on
grid on
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tPL, etaPL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#942622');
plot(tPL_L, etaPL_L*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPL_NL, etaPL_NL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#77AC30');
lg3 = legend('Experiment','$\mathrm{PL_{ellipse}}$ Model', '$\mathrm{PL_{planar}}$ Model','$\mathrm{PL_{bessel}}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')
lg3.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg3.Position; % [x, y, width, height]
lg3.Position(2) = currentPosition(2) + 0.09;
if save_fig
    save_name = strcat(fig_name,'Linearized_','P','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'P','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

figure()
hold on
grid on
plot(E_slosh_t_cut_1,abs(E_slosh_h_cut_1),'LineWidth',0.8*line_width)
plot(tPNL, etaPNL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#942622');
plot(tPNL_L, etaPNL_L*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL_NL, etaPNL_NL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#77AC30');
lg3 = legend('Experiment','$\mathrm{PNL_{ellipse}}$ Model', '$\mathrm{PNL_{planar}}$ Model','$\mathrm{PNL_{bessel}}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')
lg3.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg3.Position; % [x, y, width, height]
lg3.Position(2) = currentPosition(2) + 0.09;
if save_fig
    save_name = strcat(fig_name,'NL_','P','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'P','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

figure()
hold on
grid on
plot(tPNL_L, etaPNL_L*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL_NL, etaPNL_NL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#77AC30');
lg3 = legend('$\mathrm{PNL_{planar}}$ Model','$\mathrm{PNL_{bessel}}$ Model', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')
lg3.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg3.Position; % [x, y, width, height]
lg3.Position(2) = currentPosition(2) + 0.09;
if save_fig
    save_name = strcat(fig_name,'BesselPlanar_','P','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'P','.pdf');
    print('-dpdf', '-fillpage', save_name)
end

figure()
grid on
hold on
plot(tNL, etaNL_MSD*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#7E2F8E');
plot(tPNL_L, etaPNL_L*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#EDB120');
plot(tPNL_NL, etaPNL_NL*1000,'LineStyle',':','LineWidth',0.8*line_width,'Color','#77AC30');
lg3 = legend('$\mathrm{MSDNL_{bessel}}$','$\mathrm{PNL_{planar}}$','$\mathrm{PNL_{bessel}}$', 'Fontsize', legend_size, 'Location', 'north', 'NumColumns', num_cols, 'interpreter', 'latex');
set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex')
lg3.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg3.Position; % [x, y, width, height]
lg3.Position(2) = currentPosition(2) + 0.09;
title("Non-Linear models comparisons")

if save_fig
    save_name = strcat(fig_name,'MSD','P','.png');
    set(gcf,'PaperPositionMode','auto')
    print(save_name,'-dpng','-r0')
    save_name = strcat(fig_name,'P','.pdf');
    print('-dpdf', '-fillpage', save_name)
end