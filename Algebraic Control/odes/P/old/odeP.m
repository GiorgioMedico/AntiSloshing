function Sd = odeP(t,S,ln,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,J,g,alpha_n,wn,model_type)
% function Sd = odeP(t,S,wn,zitan,g,time,xdd_0,ydd_0,zdd_0,model_type)

%--------------------------------------------------------------------------
% ODE of Spherical Pendulum in YX parametrization (Ry(-theta)*Rx(phi_x))
%    reduces to ODE of Linear pendulum if ydd = 0 and phi(0) = 0
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------

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

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);
c_phi_x = cos(phi_x);
s_phi_x = sin(phi_x);

% %% 
% d_ = -2*zitan*wn;
% D_ = [d_, 0; 
%       0,  d_];
% 
% k_ = - wn2 - 1/ln*zdd;
% K_ = [k_, 0; 
%       0, k_];
% 
% R_ = 1/ln*[-1, 0;
%             0, -1];
% 
% %% Equations of Motion
% if strcmp(model_type,'L')
% 
%     Sd(1:2) = [dphi_y; dphi_x];
%     Sd(3:4) = D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd];
% 
% elseif strcmp(model_type,'NL')
% 
%     NLterm = cos(phi_x)*sin(phi_x);
%     % NL_ = diag([NLterm, NLterm]);
%     % A_ = [cos(phi_x)^2, 0;
%     %        0,       1];
% 
% 
%     Sd(1:2) = [dphi_y; dphi_x];
%     % Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[cos(phi_x)*sin(phi_y); cos(phi_y)*sin(phi_x)] + R_NL*[xdd; ydd] + NL_*[2*dphi_y*dphi_x; -dphi_y*dphi_y]);
% 
% 
%     Sd(3) = cos(phi_x) \ (-2*zitan*wn*dphi_y - (wn2+1/ln*zdd)*sin(phi_y) - 1/ln*xdd*cos(phi_y) + 2*sin(phi_x)*dphi_y*dphi_x);
%     Sd(4) = -2*zitan*wn*dphi_x + 1/ln*sin(phi_x)*sin(phi_y)*xdd - sin(phi_x)*cos(phi_y)*(wn2+1/ln*zdd) - 1/ln*ydd*cos(phi_x) -cos(phi_x)*sin(phi_x)*dphi_y^2;
% end


% 
d_ = -2*zitan*wn;
D_ = [d_, 0; 0, d_];

k_ = - wn2 - zdd*wn2/g;
K_ = [k_, 0; 0, k_];

% rispetto a MSD c'è 1/ln
R_ = 1/ln*[-1, 0; 0, -1];
R_NL = 1/ln*[-c_phi_x*c_phi_y, 0; s_phi_x*s_phi_y, -c_phi_x];

%% Equations of Motion
if strcmp(model_type,'L')
    A_ = eye(2);
    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd]);
elseif strcmp(model_type,'NL')
    NLterm = c_phi_x*s_phi_x;
    A_ = [c_phi_x^2, 0;
           0,       1];

    NL_ = diag([NLterm, NLterm]);
    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[c_phi_x*s_phi_y; c_phi_y*s_phi_x] + R_NL*[xdd; ydd] + NL_*[2*dphi_y*dphi_x; -dphi_y*dphi_y]);
end


end