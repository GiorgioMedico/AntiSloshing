function Sd = odePYX_rotZ(t,S,wn,zitan,g,k,J,time,xdd_0,ydd_0,zdd_0,thdd_0,model_type)

%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in YX parametrization (Ry(-phi_y)*Rx(phi_x))
% plus Rotation about Z (Rz(th))
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

% wn = n-th mode oscillation frequency (wn = sqrt(g/l))
% zitan = n-th mode damping factor
% k = viscous constant 
% J = mass moment of the liquid about the vertical axis

Sd = zeros(7,1);

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

wn2 = wn^2;
% wn2 = g/l
ln = g/wn2;


phi_y    = S(1);
phi_x    = S(2);
dphi_y   = S(3);
dphi_x   = S(4);
thl   = S(5);
thld  = S(6);
thldd = S(7);

%% Useful 

d_ = -2*zitan*wn;
D_ = [d_, 2*thld;
     -2*thld, d_];

k_ = thld^2 - (wn2 + 1/ln*zdd);

K_ = [k_, thldd;
     -thldd, k_];

% rispetto a MSD c'è 1/ln
R_ = 1/ln*[-cos(thl), -sin(thl); 
            sin(thl), -cos(thl)];

%% NL

% u1 = -(wn2+1/ln*zdd)*sin(phi_y) - 1/ln*cos(phi_y)*(cos(thl)*xdd + sin(thl)*ydd);
% damp1 = -2*zitan*wn*cos(phi_x)*dphi_y;
% nl1 = 2*sin(phi_x)*dphi_y*dphi_x + cos(phi_y)*sin(phi_x)*thldd + cos(phi_y)*cos(phi_x)*(sin(phi_y)*thld^2 + 2*thld*dphi_x);
% 
% u2 =  1/ln*(cos(thl)*sin(phi_x)*sin(phi_y) + sin(thl)*cos(phi_x))*xdd + 1/ln*(sin(thl)*sin(phi_x)*sin(phi_y) - cos(thl)*cos(phi_x))*ydd - sin(phi_x)*cos(phi_y)*(wn2+1/ln*zdd);
% damp2 = -2*zitan*wn*dphi_x;
% nl2 = -cos(phi_x)*sin(phi_x)*dphi_y^2 - sin(phi_y)*thldd + cos(phi_x)*cos(phi_y)^2*sin(phi_x)*thld^2 - 2*cos(phi_x)^2*cos(phi_y)*thld*dphi_y;

%% Equations of Motion

if strcmp(model_type,'L')

    Sd(3:4) = D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd];

elseif strcmp(model_type,'NL')

    Sd(3) = cos(phi_x) \ (-2*zitan*wn*cos(phi_x)*dphi_y - (wn2 + 1/ln*zdd)*sin(phi_y) - 1/ln*cos(phi_y)*(cos(thl)*xdd + sin(thl)*ydd) + 2*sin(phi_x)*dphi_y*dphi_x ... 
                            + cos(phi_y)*sin(phi_x)*thldd + cos(phi_y)*cos(phi_x)*(sin(phi_y)*thld^2 + 2*thld*dphi_x));

    Sd(4) = -2*zitan*wn*dphi_x + 1/ln*(cos(thl)*sin(phi_x)*sin(phi_y) + sin(thl)*cos(phi_x))*xdd + 1/ln*(sin(thl)*sin(phi_x)*sin(phi_y) - cos(thl)*cos(phi_x))*ydd ...
        - sin(phi_x)*cos(phi_y)*(wn2+1/ln*zdd) - cos(phi_x)*sin(phi_x)*dphi_y^2 - sin(phi_y)*thldd + cos(phi_x)*cos(phi_y)^2*sin(phi_x)*thld^2 - 2*cos(phi_x)^2*cos(phi_y)*thld*dphi_y;

end

Sd(1:2) = [dphi_y; dphi_x];
Sd(5:6) = [thld; thldd];
Sd(7) = k/J*(thdd - thldd);


end