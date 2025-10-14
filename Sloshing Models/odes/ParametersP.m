function [g, density, m_tot, V, csi11, Cs, ms, ks, cs, as, l, L, J, k, wn] = ParametersP(R, h)
    % Parameters - Function that returns a vector of the fluid parameters
    %
    % Syntax:
    %   output = Parameters(R, h)
    %
    % Inputs:
    %   R - Radius of cylindrical container in meters
    %   h - Height of liquid in meters
    % Outputs:
    %   Parameters of the fluid
    %   1 = gravity
    %   2 = density of water
    %   3 = total mass of the fluid
    %   4 = volume of fluid
    %   5 = bessel derivative 1_1
    %   6 = damping factor
    %   7 = sloshing mass
    %   8 = slosh elastic constant
    %   9 = slosh damping
    %   10 = non-linear spring constant
    %   11 = pendulum length
    %   12 = inertia of the liquid
    %   13 = liquid drag constant
    %   14 = frequency 

    %% General

    g = 9.80665;
    % v = 1.004 * 10^-6; %dynamic viscosity of water
    mu = 0.001;       %viscosità acqua
    density = 998.2071; %density of water

    csi11 = 1.84118; %bessel zero derivative 1_1
    % b = 0.08; %damping between liquid and container 

    V = pi*R^2*h; %volume of a cylinder
    m_tot = density * V; %total mass of the liquid
    J = 0.5*density*V*R^2;    %liquid inertia

    %% Pendulum

    wn = sqrt(g*csi11/R * tanh(h*csi11/R)); %natural frequency of the fundamental sloshing mode
    delta = 2.89/pi * sqrt(mu/sqrt(R^3 *g)) * (1 + (0.318/sinh(1.84*h/R))*((1-h/R)/cosh(1.84*h/R))); %damping of first mode
    l = (R/csi11)*tanh(csi11*h/R); %length of the pendulum arm
    ms = ((2*m_tot*R)/(csi11 * h * (csi11^2 - 1)))*tanh(csi11*h/R); %mass of the sloshing mass
    L = -2*R/(csi11*sinh(csi11*h/R));
    % paper "A simple model base...
    
    %% Mass-Spring-Damper
    Cs = 0.92*(sqrt((mu/density)/sqrt(g*(R^3))))*(1+(0.318/sinh(1.84*h/R))*(1+(1-h/R)/cosh(1.84*h/R)));  %fattore smorzamento liquido (primo modo)
    
    % ms = ((2*m_tot*R)/(csi11 * h * (csi11^2 - 1)))*tanh(csi11*h/R); 
    k = 2*log(10)*J*mu/(R^2*density);    %costante trascinamento liquido
    ks = ms*g*(csi11/R)*tanh(csi11*h/R);  %costante elastica molla
    cs = 2*Cs*sqrt(ks*ms); %damping
    as=0.58;         %costante molla non lineare


end

