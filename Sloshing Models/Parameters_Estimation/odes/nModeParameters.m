function [g, rho, mF, V, csi1n, zitan, mn, kn, cn, alphan, ln, Ln, J, k, wn] = nModeParameters(R, h, n)
    % Parameters - Function that returns a vector of the fluid parameters
    %
    % Syntax:
    %   output = Parameters(R, h)
    %
    % Inputs:
    %   R - Radius of cylindrical container in meters
    %   h - Height of liquid in meters
    %   n - n-th sloshing mode
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
    rho = 998.2071; %density of water
    
    if n == 1
        csi1n = 1.84118; %bessel zero derivative 1_1  8.53632 
    elseif n == 2
        csi1n = 5.33144;
    elseif n == 3
        csi1n = 8.53632;
    end
    % b = 0.08; %damping between liquid and container 

    V = pi*R^2*h; %volume of a cylinder
    mF = rho * V; %total mass of the liquid
    J = 0.5*rho*V*R^2;    %liquid inertia

    %% Pendulum

    wn = sqrt(g*csi1n/R * tanh(h*csi1n/R)); %natural frequency of the fundamental sloshing mode
    delta = 2.89/pi * sqrt(mu/sqrt(R^3 *g)) * (1 + (0.318/sinh(csi1n*h/R))*((1-h/R)/cosh(csi1n*h/R))); %damping of first mode
    % ln = (R/csi1n)*tanh(csi1n*h/R); %length of the pendulum arm
    ln = (R/csi1n)/tanh(csi1n*h/R); %length of the pendulum arm
    mn = ((2*mF*R)/(csi1n * h * (csi1n^2 - 1)))*tanh(csi1n*h/R); %mass of the sloshing mass
    Ln = -2*R/(csi1n*sinh(2*csi1n*h/R));
    % paper "A simple model base...
    
    %% Mass-Spring-Damper
    zitan = 0.92*(sqrt((mu/rho)/sqrt(g*(R^3))))*(1+(0.318/sinh(csi1n*h/R))*(1+(1-h/R)/cosh(csi1n*h/R)));  %fattore smorzamento liquido (primo modo)
    
    % ms = ((2*m_tot*R)/(csi11 * h * (csi11^2 - 1)))*tanh(csi11*h/R); 
    k = 2*log(10)*J*mu/(R^2*rho);    %costante trascinamento liquido
    kn = mn*g*(csi1n/R)*tanh(csi1n*h/R);  %costante elastica molla
    cn = 2*zitan*sqrt(kn*mn); %damping
    alphan=0.58;         %costante molla non lineare


end

