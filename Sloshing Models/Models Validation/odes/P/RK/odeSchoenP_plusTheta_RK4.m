function Sd = odeSchoenP_plusTheta_RK4(S,wn,zitan,xdd,ydd,zdd,th,thd,thdd,g,model_type)

%--------------------------------------------------------------------------
% ODE Runge-Kutta of Spherical Pendulum in YX parametrization (Ry(+phi_y)*Rx(phi_x))
% plus Rotation about Z (Rz(th))
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       July 2025
%--------------------------------------------------------------------------

% wn = n-th mode oscillation frequency (wn = sqrt(g/l))
% zitan = n-th mode damping factor

Sd = zeros(4,1);

wn2 = wn^2; % wn2 = g/l
ln = g/wn2;

phi_y    = S(1);
phi_x    = S(2);
dphi_y   = S(3);
dphi_x   = S(4);

%% Useful 

d_ = -2*zitan*wn;
D_ = [d_, 2*thd;
     -2*thd, d_];

k_ = thd^2 - (wn2 + 1/ln*zdd);

K_ = [k_, thdd;
     -thdd, k_];

% rispetto a MSD c'è 1/ln
R_ = 1/ln*[-cos(th), -sin(th); 
            sin(th), -cos(th)];

%% NL

% u1 = -(wn2+1/ln*zdd)*sin(phi_y) - 1/ln*cos(phi_y)*(cos(th)*xdd + sin(th)*ydd);
% damp1 = -2*zitan*wn*cos(phi_x)*dphi_y;
% nl1 = 2*sin(phi_x)*dphi_y*dphi_x + cos(phi_y)*sin(phi_x)*thdd + cos(phi_y)*cos(phi_x)*(sin(phi_y)*thd^2 + 2*thd*dphi_x);
% 
% u2 =  1/ln*(cos(th)*sin(phi_x)*sin(phi_y) + sin(th)*cos(phi_x))*xdd + 1/ln*(sin(th)*sin(phi_x)*sin(phi_y) - cos(th)*cos(phi_x))*ydd - sin(phi_x)*cos(phi_y)*(wn2+1/ln*zdd);
% damp2 = -2*zitan*wn*dphi_x;
% nl2 = -cos(phi_x)*sin(phi_x)*dphi_y^2 - sin(phi_y)*thdd + cos(phi_x)*cos(phi_y)^2*sin(phi_x)*thd^2 - 2*cos(phi_x)^2*cos(phi_y)*thd*dphi_y;

%% Equations of Motion

if strcmp(model_type,'L')

    Sd(3:4) = D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd];

elseif strcmp(model_type,'NL')

    Sd(3) = cos(phi_x) \ (-2*zitan*wn*cos(phi_x)*dphi_y - (wn2 + 1/ln*zdd)*sin(phi_y) + 1/ln*cos(phi_y)*(cos(th)*xdd + sin(th)*ydd) + 2*sin(phi_x)*dphi_y*dphi_x ... 
                            - cos(phi_y)*sin(phi_x)*thdd + cos(phi_y)*cos(phi_x)*(sin(phi_y)*thd^2 - 2*thd*dphi_x));

    Sd(4) = -2*zitan*wn*dphi_x + 1/ln*(- cos(th)*sin(phi_x)*sin(phi_y) + sin(th)*cos(phi_x))*xdd - 1/ln*(sin(th)*sin(phi_x)*sin(phi_y) + cos(th)*cos(phi_x))*ydd ...
        - sin(phi_x)*cos(phi_y)*(wn2+1/ln*zdd) - cos(phi_x)*sin(phi_x)*dphi_y^2 + sin(phi_y)*thdd + cos(phi_x)*cos(phi_y)^2*sin(phi_x)*thd^2 + 2*cos(phi_x)^2*cos(phi_y)*thd*dphi_y;

end

Sd(1:2) = [dphi_y; dphi_x];

end