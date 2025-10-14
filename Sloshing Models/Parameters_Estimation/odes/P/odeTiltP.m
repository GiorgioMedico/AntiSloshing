function Sd = odeTiltP(t,S,ln,Ln,zitan,mn,time,xdd_0,ydd_0,zdd_0,psi_0,psid_0,psidd_0,h,hn,g,alpha_n,wn,model_type)

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

dn = h - Ln;

%% Useful 
d_ = 2*zitan*wn;
D_ = [d_, 0; 
      0,  d_];

k_ = - 1/ln*(xdd*sin(psi) + zdd*cos(psi)) - wn2*cos(psi) + dn/ln*psid^2;

K_ = [k_ , 0; 0, k_ - psid^2];
R_ = 1/ln*[ (ln - dn)*psidd - (xdd*cos(psi) - zdd*sin(psi)) + g*sin(psi); - ydd];

%% NL

% u1 = - 1/ln*xdd*cos(psi-phi_y) + k_*sin(psi-phi_y);
% damp1 = -d_*cos(phi_x)*dphi_y;
% nl1 = dn/ln * sin(phi_y)*psid^2 + 1/ln * (ln*cos(phi_x) - dn*cos(phi_y))*psidd - 2*sin(phi_x)*dphi_x*(psid - dphi_y);
% 
% u2 = - 1/ln*sin(phi_x)*sin(psi-phi_y)*xdd - sin(phi_x)*cos(psi-phi_y)*k_ - 1/ln*ydd*cos(phi_x);
% damp2 = -d_*dphi_x;
% nl2 = - 1/ln*sin(phi_x)*(ln*cos(phi_x) - dn*cos(phi_y))*psid^2 + dn/ln *sin(phi_x)*sin(phi_y)*psidd - sin(phi_x)*cos(phi_x)*dphi_y^2 + 2*sin(phi_x)*cos(phi_x)*psid*dphi_y;

%% Equations of Motion
Sd(1:2) = [dphi_y; dphi_x];

if strcmp(model_type,'L')

    % Sd(3:4) = D_*[dphi_y; dphi_x] + K_*[phi_y; phi_x] + R_;
    Sd(3) = -d_*dphi_y - 1/ln*cos(psi)*xdd + 1/ln*sin(psi)*(zdd+g) - 1/ln*(sin(psi)*xdd + cos(psi)*(zdd+g))*phi_y + 1/ln*(ln-dn)*psidd + dn/ln*phi_y*psid^2;
    Sd(4) = - d_*dphi_x - 1/ln*(sin(psi)*xdd + cos(psi)*(zdd+g))*phi_x -1/ln*ydd - 1/ln*(ln-dn)*phi_x*psid^2; 


elseif strcmp(model_type,'NL')

    % Sd(3) = cos(phi_x) \ (-d_*cos(phi_x)*dphi_y + k_*sin(psi-phi_y) - 1/ln*xdd*cos(psi-phi_y) + dn/ln * sin(phi_y)*psid^2 + 1/ln * (ln*cos(phi_x) - dn*cos(phi_y))*psidd - 2*sin(phi_x)*dphi_x*(psid - dphi_y));
    Sd(3) = cos(phi_x) \ (-d_*cos(phi_x)*dphi_y - 1/ln*cos(psi-phi_y)*xdd + 1/ln*sin(psi-phi_y)*(zdd+g) + 1/ln*(ln*cos(phi_x)-dn*cos(phi_y))*psidd - 2*sin(phi_x)*dphi_x*(psid-dphi_y) + dn/ln*sin(phi_y)*psid^2);


    % Sd(4) = -d_*dphi_x - 1/ln*sin(phi_x)*sin(psi-phi_y)*xdd - sin(phi_x)*cos(psi-phi_y)*k_ - 1/ln*ydd*cos(phi_x) - 1/ln*sin(phi_x)*(ln*cos(phi_x) - dn*cos(phi_y))*psid^2 ...
    %         + dn/ln *sin(phi_x)*sin(phi_y)*psidd - sin(phi_x)*cos(phi_x)*dphi_y^2 + 2*sin(phi_x)*cos(phi_x)*psid*dphi_y;
    Sd(4) = - d_*dphi_x - 1/ln*sin(phi_x)*sin(psi-phi_y)*xdd - 1/ln*sin(phi_x)*cos(psi-phi_y)*(zdd+g)-1/ln*cos(phi_x)*ydd - 1/ln*sin(phi_x)*(ln*cos(phi_x)-dn*cos(phi_y))*psid^2 + dn/ln*sin(phi_x)*sin(phi_y)*psidd+...
            - sin(phi_x)*cos(phi_x)*dphi_y^2 + 2*sin(phi_x)*cos(phi_x)*psid*dphi_y;
end


end