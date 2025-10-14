function [g, m_tot, V, csi11, Cs, ms, ks, cs, as, l, J, k, wn] = Parameters(R, h, density, mu)
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

    %% General

    g = 9.81;
    % v = 1.004 * 10^-6; %dynamic viscosity of water

    csi11 = 1.84118; %bessel zero derivative 1_1
    % b = 0.08; %damping between liquid and container 

    V = pi*R^2*h; %volume of a cylinder
    m_tot = density * V; %total mass of the liquid
    J = 0.5*density*V*R^2;    %liquid inertia

    %% Pendulum

    wn = sqrt(g*csi11/R * tanh(h*csi11/R)); %natural frequency of the fundamental sloshing mode
    delta = 2.89/pi * sqrt(mu/sqrt(R^3 *g)) * (1 + (0.318/sinh(1.84*h/R))*((1-h/R)/cosh(1.84*h/R))); %damping of first mode
    l = g / wn^2; %length of the pendulum arm
    %ms = ((2*m_tot*R)/(csi11 * h * (csi11^2 - 1)))*tanh(csi11*h/R); %mass of the sloshing mass
    % paper "A simple model base...
    
    %% Mass-Spring-Damper
    Cs = 0.92*(sqrt((mu/density)/sqrt(g*(R^3))))*(1+(0.318/sinh(1.84*h/R))*(1+(1-h/R)/cosh(1.84*h/R)));  %fattore smorzamento liquido (primo modo)     %FORMULA 3.3 (ZITA S)
    
    ms = ((2*m_tot*R)/(csi11 * h * (csi11^2 - 1)))*tanh(csi11*h/R); %FORMULA 3.2
    k = 2*log(10)*J*mu/(R^2*density);    %costante trascinamento liquido  %FORMULA 3.24
    ks = ms*g*(csi11/R)*tanh(csi11*h/R);  %costante elastica molla %DA FORMULA 3.1
    cs = 2*Cs*sqrt(ks*ms); %damping  %%FORMULA SCRITTA SOPRA LA 3.3
    as=0;         %costante molla non lineare  %as è alfa_s CAPITOLO 3.3.2


end



