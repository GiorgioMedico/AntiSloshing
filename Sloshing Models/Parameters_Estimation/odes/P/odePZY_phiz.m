function Sd = odePZY_phiz(t,S,wn,zitan,g,time,xdd_0,ydd_0,zdd_0,phi_z_0,dphi_z_0,model_type)
%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in ZY parametrization (Rz(phi)*Ry(-theta)) with
%                phi_z calculated algebraically and feeded to ode
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
   ydd  = spline(time,ydd_0,t);
   zdd  = spline(time,zdd_0,t);
   phi_z  = spline(time,phi_z_0,t);
   dphi_z = spline(time,dphi_z_0,t);
else
   xdd  = 0;
   ydd  = 0;
   zdd  = 0;
   phi_z = 0;
   dphi_z = 0;
end 

phi_y    = S(1);
dphi_y   = S(2);

%% Useful 
d_ = 2*zitan*wn;

k_ = wn2 + 1/ln*zdd;

%% Equations of motion
Sd(1) = dphi_y;

if strcmp(model_type,'L')

    Sd(2) = - d_*dphi_y - k_*phi_y - 1/ln*(xdd + phi_z*ydd);

elseif strcmp(model_type,'NL')
        
    Sd(2) = - d_*dphi_y  - k_*sin(phi_y) - 1/ln*cos(phi_y)*(cos(phi_z)*xdd + sin(phi_z)*ydd) + cos(phi_y)*sin(phi_y)*dphi_z^2;

end

end