function Sd = odePZY_rotZ(t,S,wn,zitan,g,k,J,time,xdd_0,ydd_0,zdd_0,thdd_0,model_type)
%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in ZY parametrization (Rz(phi)*Ry(-theta))
% plus Rotation about Z
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

% wn = n-th mode oscillation frequency (wn = sqrt(g/l))
% zitan = n-th mode damping factor
% k = viscous constant 
% J = mass moment of the liquid about the vertical axis

Sd = zeros(6,1);

wn2 = wn^2;
ln = g/wn2;

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   ydd  = spline(time,ydd_0,t);
   zdd  = spline(time,zdd_0,t);
   thdd = spline(time,thdd_0,t);
else
   xdd  = 0;
   ydd  = 0;
   zdd  = 0;
   thdd = 0;
end 

phi_y    = S(1);
phi_z    = S(2);
dphi_y   = S(3);
dphi_z   = S(4);
thl   = S(5);
thld  = S(6);
thldd = S(7);

%% Useful 
d_ = 2*zitan*wn;

k_ = wn2 + 1/ln*zdd;

%% Equations of motion

if strcmp(model_type,'L')

    disp("Not yet implemented");

elseif strcmp(model_type,'NL')
        
    Sd(3) = - d_*dphi_y  - k_*sin(phi_y) - 1/ln*cos(phi_y)*(cos(phi_z+thl)*xdd + sin(phi_z+thl)*ydd) + cos(phi_y)*sin(phi_y)*(dphi_z+thld)^2;
    Sd(4) = sin(phi_y) \ (-thldd*sin(phi_y) - d_*dphi_z*sin(phi_y) + 1/ln*(sin(phi_z+thl)*xdd - cos(phi_z+thl)*ydd) - 2*cos(phi_y)*dphi_y*(dphi_z+thld));
end

Sd(1:2) = [dphi_y; dphi_z];
Sd(5:6) = [thld; thldd];
Sd(7) = k/J*(thdd - thldd);

end