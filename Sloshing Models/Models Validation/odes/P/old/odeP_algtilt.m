function Sd = odeP_algtilt(t,S,ln,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,phi_z,dphi_z,ddphi_z,beta_y,dbeta_y,ddbeta_y,J,g,alpha_n,wn,model_type)
%% pendulum zy parametrization with phi_z calculated analytically
% phi_z = pi+atan2(ay,ax) because assuming phi_y and its derivatives to
% zero (we want to completely avoid sloshing), we have -sin(phi_z)*ax +
% cos(phi_z)*ay = 0

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   ydd  = spline(time,ydd_0,t);
   zdd  = spline(time,zdd_0,t);
   phi_z  = spline(time,phi_z,t);
   beta_y = spline(time,beta_y,t);
   dphi_z  = spline(time,dphi_z,t);
   dbeta_y = spline(time,dbeta_y,t);
   ddphi_z  = spline(time,ddphi_z,t);
   ddbeta_y = spline(time,ddbeta_y,t);
else
   xdd  = 0;
   ydd  = 0;
   zdd  = 0;
   phi_z = 0;
   beta_y = 0;
   dphi_z = 0;
   dbeta_y = 0;
   ddphi_z = 0;
   ddbeta_y = 0;
end 

wn = sqrt(g/ln);
wn2 = wn^2;
wn4 = wn^4;

% ln = g/wn2;

g2 = g^2;

Sd = zeros(2,1);

phi_y    = S(1);
dphi_y   = S(2);
phi_y2 = phi_y^2;
dphi_y2 = dphi_y^2;

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);

c_beta_y = cos(beta_y);
s_beta_y = sin(beta_y);

d_ = -2*zitan*wn;
k_ = - wn2 - zdd*wn2/g;

H = ln;

if strcmp(model_type,'L')
    % Sd(2) = d_*dphi_y + k_*phi_y -1/ln*xdd*cos(phi_z) -1/ln*ydd*sin(phi_z);
    % Sd(2) = 1/ln*(s_beta_y*phi_y - c_beta_y*cos(phi_z))*xdd - 1/ln*sin(phi_z) - 1/ln*(c_beta_y*phi_y+s_beta_y*cos(phi_z))*(zdd+g)+((H/ln - 1/2 + 1/2*cos(2*phi_z))*phi_y)*dbeta_y^2 - 1/ln*(ln-H)*cos(phi_z)*ddbeta_y + phi_y*dphi_z - d_*dphi_y;

    Sd(2) = 1/ln*(s_beta_y*phi_y - c_beta_y*cos(phi_z))*xdd -1/ln*ydd*sin(phi_z) - 1/ln*(c_beta_y*phi_y+s_beta_y*cos(phi_z))*(zdd+g)+((H/ln - 1/2 + 1/2)*phi_y)*dbeta_y^2 - 1/ln*(ln-H)*ddbeta_y - d_*dphi_y;
elseif strcmp(model_type,'NL')
    error("not available")
end
Sd(1) = dphi_y;

end