function Sd = odeSchoenP(t,S,ln,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,thdd_0,J,g,alpha_n,wn,model_type)

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

% wn = sqrt(g/ln);
wn2 = wn^2;
wn4 = wn^4;

ln = g/wn2;

g2 = g^2;

Sd = zeros(6,1);


phi_y    = S(1);
phi_x    = S(2);
dphi_y   = S(3);
dphi_x   = S(4);
thl   = S(5);
thld  = S(6);
thldd = S(7);



% xn    = S(1);
% yn    = S(2);
% xnd   = S(3);
% ynd   = S(4);
% thl   = S(5);
% thld  = S(6);

phi_y2 = phi_y^2;
phi_x2 = phi_x^2;
dphi_y2 = dphi_y^2;
dphi_x2 = dphi_x^2;

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);
c_phi_x = cos(phi_x);
s_phi_x = sin(phi_x);

d_ = -2*zitan*wn;
D_ = [d_, 2*thld; -2*thld, d_];
D_NL = [1, c_phi_y*c_phi_x^2; c_phi_y*c_phi_x^2, 1]*D_;

k_ = thld^2 - wn2 - zdd*wn2/g;
k_NL1 = c_phi_y*s_phi_y*c_phi_x^2*thld^2 - s_phi_y*c_phi_x*(wn2 + zdd*wn2/g);
k_NL2 = c_phi_x*s_phi_x*c_phi_y^2*thld^2 - c_phi_y*s_phi_x*(wn2 + zdd*wn2/g);

K_ = [k_, thldd; -thldd, k_];
K_NL = [k_NL1, c_phi_x*s_phi_x*c_phi_y*thldd; -s_phi_y*thldd, k_NL2];

% rispetto a MSD c'è 1/ln
R_ = 1/ln*[-cos(thl), -sin(thl); 
            sin(thl), -cos(thl)];
R_NL = [c_phi_x*c_phi_y, 0;
        -s_phi_y*s_phi_x, c_phi_x]*R_;


if strcmp(model_type,'L')
    NLterm = 0;
    A_ = eye(2);

    Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd]);

elseif strcmp(model_type,'NL')
    NLterm = c_phi_x*s_phi_x;
    A_ = [c_phi_x^2, 0;
           0,       1];

    NL_ = diag([NLterm, NLterm]);

    Sd(3:4) = A_\(D_NL*[dphi_y; dphi_x] + K_NL*[1;1] + R_NL*[xdd; ydd] + NL_*[2*dphi_y*dphi_x; -dphi_y*dphi_y]);
end
% NL_ = diag([NLterm, NLterm]);
% 
% Sd(1:2) = [dphi_y; dphi_x];
% Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_*[xdd; ydd] + NL_*[xn; yn]);
% Sd(5:6) = [thld; thdd];
Sd(1:2) = [dphi_y; dphi_x];
Sd(5:6) = [thld; thldd];
Sd(7) = k/J*(thdd - thldd);


end