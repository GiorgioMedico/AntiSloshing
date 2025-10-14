function Sd = odePYX(t,S,wn,zitan,g,time,xdd_0,ydd_0,zdd_0,model_type)

%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in YX parametrization (Ry(-phi_y)*Rx(phi_x))
%    reduces to ODE of Linear pendulum if ydd = 0 and phi(0) = 0
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

% wn = n-th mode oscillation frequency
% zitan = n-th mode damping factor

Sd = zeros(4,1);

wn2 = wn^2;
ln = g/wn2;

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   ydd  = spline(time,ydd_0,t);
   zdd  = spline(time,zdd_0,t);
else
   xdd  = 0;
   ydd  = 0;
   zdd  = 0;
end 

phi_y    = S(1);
phi_x    = S(2);
dphi_y   = S(3);
dphi_x   = S(4);

%% Useful 
d_ = 2*zitan*wn;
D_ = [d_, 0; 
      0,  d_];

k_ = wn2 + 1/ln*zdd;
K_ = [k_, 0; 
      0, k_];

R_ = 1/ln*[1, 0;
            0, 1];

%% Equations of Motion
Sd(1:2) = [dphi_y; dphi_x];

if strcmp(model_type,'L')

    Sd(3:4) = - D_*[dphi_y; dphi_x] - K_*[phi_y; phi_x] - R_*[xdd; ydd];

elseif strcmp(model_type,'NL')

    Sd(3) = cos(phi_x) \ (-d_*dphi_y - k_*sin(phi_y) - 1/ln*xdd*cos(phi_y) + 2*sin(phi_x)*dphi_y*dphi_x);
    Sd(4) = -d_*dphi_x + 1/ln*sin(phi_x)*sin(phi_y)*xdd - sin(phi_x)*cos(phi_y)*k_ - 1/ln*ydd*cos(phi_x) -cos(phi_x)*sin(phi_x)*dphi_y^2;
end

end