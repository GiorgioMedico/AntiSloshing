function Sd = odeP_ZY1(t,S,ln,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,J,g,alpha_n,wn,model_type)

if t >= 0 && t <= time(end)
   xdd  = spline(time,xdd_0,t);
   ydd  = spline(time,ydd_0,t);
   zdd  = spline(time,zdd_0,t);
   % phi_z  = spline(time,phi_z,t);

   % phi_z = atan2(ydd,xdd);
   % if phi_z<0
        % phi_z = pi+phi_z;
   % end
   
else
   xdd  = 0;
   ydd  = 0;
   zdd  = 0;
   % phi_z = 0;
end 

% disp(phi_z);

wn = sqrt(g/ln);
wn2 = wn^2;
wn4 = wn^4;

% ln = g/wn2;

g2 = g^2;

Sd = zeros(4,1);


phi_y    = S(1);
phi_z    = S(2);
dphi_y   = S(3);
dphi_z   = S(4);


phi_y2 = phi_y^2;
phi_z2 = phi_z^2;
dphi_y2 = dphi_y^2;
dphi_z2 = dphi_z^2;

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);
c_phi_z = cos(phi_z);
s_phi_z = sin(phi_z);

d_ = -2*zitan*wn;
D_ = [d_, 0;
      0 , d_*sin(phi_y)];

k_ = - wn2 - zdd*wn2/g;
K_ = [k_, 0; 0, k_];

k_NL1 = - s_phi_y*(wn2 + zdd*wn2/g);
k_NL2 = 0;

% K_NL = [k_NL1, c_phi_x*s_phi_x*c_phi_y*thldd; -s_phi_y*thldd, k_NL2];

K_NL = [k_NL1; k_NL2];


% rispetto a MSD c'è 1/ln
% R_ = 1/ln*[-cos(thl), -sin(thl); sin(thl), -cos(thl)];
R_NL = 1/ln*[-c_phi_z*c_phi_y, -s_phi_z*c_phi_y; 
        +s_phi_z, -c_phi_z];

if strcmp(model_type,'L')
    
    Sd(3) = d_*dphi_y + k_*phi_y -1/ln*xdd -1/ln*ydd*sin(phi_z);
    Sd(4) = 0;
elseif strcmp(model_type,'NL')
    NLterm = c_phi_y;
    % A_ = [1, 0;
    %        0,       sin(phi_y)];
    A_ = [1, 0;
           0,       sin(phi_y)];

    NL_ = diag([NLterm*s_phi_y, NLterm]);

    Sd(3:4) = A_\(D_*[dphi_y; dphi_z] + K_NL + R_NL*[xdd; ydd] + NL_*[dphi_z*dphi_z; -2*dphi_z*dphi_y]);
end
% NL_ = diag([NLterm, NLterm]);
% 
Sd(1:2) = [dphi_y; dphi_z];
% Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd] + NL_*[xn; yn]);
% Sd(5:6) = [thld; thdd];
% Sd(1) = dphi_y;

end