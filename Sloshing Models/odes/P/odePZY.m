function Sd = odePZY(t,S,wn,zitan,g,time,xdd_0,ydd_0,zdd_0,model_type)
%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in ZY parametrization (Rz(phi)*Ry(-theta))
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
phi_z    = S(2);
dphi_y   = S(3);
dphi_z   = S(4);

%% Useful 
d_ = 2*zitan*wn;

k_ = wn2 + 1/ln*zdd;

%% Equations of motion
Sd(1:2) = [dphi_y; dphi_z];

if strcmp(model_type,'L')

    Sd(3) = - d_*dphi_y - k_*phi_y - 1/ln*(xdd + phi_z*ydd) ;
    Sd(4) = phi_y \ (- d_*dphi_z*phi_y + 1/ln*(phi_z*xdd - ydd));

elseif strcmp(model_type,'NL')
        
    Sd(3) = - d_*dphi_y  - k_*sin(phi_y) - 1/ln*cos(phi_y)*(cos(phi_z)*xdd + sin(phi_z)*ydd) + cos(phi_y)*sin(phi_y)*dphi_z^2;
    Sd(4) = sin(phi_y) \ (- d_*dphi_z*sin(phi_y) + 1/ln*(sin(phi_z)*xdd - cos(phi_z)*ydd) - 2*cos(phi_y)*dphi_y*dphi_z);
end

end