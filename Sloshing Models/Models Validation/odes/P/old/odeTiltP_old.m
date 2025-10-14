function Sd = odeTiltP_old(t,S,ln,Ln,zitan,mn,time,xdd_0,ydd_0,zdd_0,psi_0,psid_0,psidd_0,h,hn,g,alpha_n,wn,model_type)

if t >= 0 && t <= time(end)
   xdd   = spline(time,xdd_0,t);
   ydd   = spline(time,ydd_0,t);
   zdd   = spline(time,zdd_0,t);
   psi   = spline(time,psi_0,t);
   psid  = spline(time,psid_0,t);
   psidd = spline(time,psidd_0,t);
else
   xdd   = 0;
   ydd   = 0;
   zdd   = 0;
   psi   = 0;
   psid  = 0;
   psidd = 0;
end 

wn = sqrt(g/ln);
wn2 = wn^2;
wn4 = wn^4;

g2 = g^2;

Sd = zeros(4,1);

phi_y    = S(1);
phi_x    = S(2);
dphi_y   = S(3);
dphi_x   = S(4);

phi_y2 = phi_y^2;
phi_x2 = phi_x^2;
dphi_y2 = dphi_y^2;
dphi_x2 = dphi_x^2;

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);
c_phi_x = cos(phi_x);
s_phi_x = sin(phi_x);

H = h - Ln;

d_ = -2*zitan*wn;
D_ = [d_, 0; 0, d_];
D_NL = [d_, -2*c_phi_x*s_phi_x*psid; 2*c_phi_x*s_phi_x*psid, d_];

k_ = - wn2/g*(xdd*sin(psi) + zdd*cos(psi)) - wn2*cos(psi) + psid^2*wn2/g*H;
k_NL1 = - wn2/g*s_phi_y*(xdd*sin(psi) + zdd*cos(psi)) + (- wn2*cos(psi) + psid^2*wn2/g*H)*c_phi_x*s_phi_y;
k_NL2 = - wn2/g*s_phi_x*c_phi_y*(xdd*sin(psi) + zdd*cos(psi)) + (- wn2*cos(psi) + psid^2*wn2/g*H)*s_phi_x*c_phi_y;

K_ = [k_ , 0; 0, k_ - psid^2];
K_NL = [k_NL1 , 0; 0, k_NL2 - s_phi_x*c_phi_x*psid^2];

R_ = 1/ln*[ (ln - H)*psidd - (xdd*cos(psi) - zdd*sin(psi)) + g*sin(psi);
                              - ydd];
R_NL = 1/ln*[ (ln*c_phi_x - H*c_phi_y)*c_phi_x*psidd - c_phi_x*c_phi_y*(xdd*cos(psi) - zdd*sin(psi)) + g*sin(psi)*c_phi_x*c_phi_y ;
                              - ydd*c_phi_x + s_phi_x*s_phi_y*(H*psidd - xdd*cos(psi) - (g+zdd)*sin(psi))];

if strcmp(model_type,'L')
    A_ = eye(2);


    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3:4) = A_\(D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_);
elseif strcmp(model_type,'NL')
    NLterm = c_phi_x*s_phi_x;
    A_ = [c_phi_x^2, 0;
           0,       1];

    NL_ = diag([NLterm, NLterm]);

    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3:4) = A_\(D_NL*[dphi_y; dphi_x] + K_NL*[1;1] + R_NL + NL_*[2*dphi_y*dphi_x; -dphi_y*dphi_y]);
end



end