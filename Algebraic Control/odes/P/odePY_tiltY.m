function Sd = odePY_tiltY(t,S,wn,zitan,g,dn,time,xdd_0,zdd_0,psi_0,psid_0,psidd_0,model_type)

%--------------------------------------------------------------------------
% ODE of Linear Pendulum in Y parametrization (Ry(-phi_y))
%       with tilting about Y axis (Ry(-psi))
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

% wn = n-th mode oscillation frequency
% zitan = n-th mode damping factor
% dn = distance from center of rotation to pivot (e.g. if rotation is about
% container base dn = h - Ln, if about pendulum mass dn = ln)

Sd = zeros(2,1);

wn2 = wn^2;
ln = g/wn2;

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   zdd  = spline(time,zdd_0,t);
   psi   = spline(time,psi_0,t);
   psid  = spline(time,psid_0,t);
   psidd = spline(time,psidd_0,t);
else
   xdd  = 0;
   zdd  = 0;
   psi   = 0;
   psid  = 0;
   psidd = 0;
end 

phi_y    = S(1);
dphi_y   = S(2);

%% Useful 
d_ = 2*zitan*wn;
k_ = wn2 + 1/ln*zdd; % wn2 = g/ln

%% Equations of Motion
Sd(1) = dphi_y;
if strcmp(model_type,'L')

    Sd(2) = -d_*dphi_y - k_*(sin(psi)+phi_y*cos(psi)) - 1/ln*(cos(psi)-phi_y*sin(psi))*xdd + dn/ln*phi_y*psid^2 - 1/ln*(ln - dn)*psidd;

elseif strcmp(model_type,'NL')

    Sd(2) = -d_*dphi_y - k_*sin(psi+phi_y) - 1/ln*cos(psi+phi_y)*xdd + dn/ln*sin(phi_y)*psid^2 - 1/ln*(ln - dn*cos(phi_y))*psidd;

end

end

