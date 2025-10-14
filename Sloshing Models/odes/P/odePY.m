function Sd = odePY(t,S,wn,zitan,g,time,xdd_0,zdd_0,model_type)

%--------------------------------------------------------------------------
% ODE of Linear Pendulum in Y parametrization (Ry(-theta))
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

Sd = zeros(2,1);

wn2 = wn^2;
ln = g/wn2;

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   zdd  = spline(time,zdd_0,t);
else
   xdd  = 0;
   zdd  = 0;
end 

phi_y    = S(1);
dphi_y   = S(2);

%% Useful 
d_ = 2*zitan*wn;
k_ = wn2 + 1/ln*zdd;

%% Equations of Motion
Sd(1) = dphi_y;
if strcmp(model_type,'L')

    Sd(2) = -d_*dphi_y - k_*phi_y - 1/ln*xdd;

elseif strcmp(model_type,'NL')

    Sd(2) = -d_*dphi_y - k_*sin(phi_y) - 1/ln*cos(phi_y)*xdd;

end

end