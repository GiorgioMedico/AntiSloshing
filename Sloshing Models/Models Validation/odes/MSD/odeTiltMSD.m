function Sd = odeTiltMSD(t,S,kn,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,psi_0,psid_0,psidd_0,h,hn,g,alpha_n,w,model_type)

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

wn = sqrt(kn/mn);
wn2 = wn^2;
wn4 = wn^4;

g2 = g^2;

Sd = zeros(4,1);

xn    = S(1);
yn    = S(2);
xnd   = S(3);
ynd   = S(4);

xn2 = xn^2;
yn2 = yn^2;
xnd2 = xnd^2;
ynd2 = ynd^2;

if strcmp(model_type,'L')
    NLterm1 = 0;
    NLterm2 = 0;
    H = h/2 + hn;
    A_ = eye(2);
elseif strcmp(model_type,'NL')
    NLterm1 = -wn4/g2*(xnd2 + ynd2) - 2*zitan*wn*wn4/g2*(xn*xnd + yn*ynd) - wn2*alpha_n*(xn2 + yn2)^(w-1) + psidd*wn2/g*xn ;
    NLterm2 = -2*psid*wn2/g*yn;
    H = h/2 + hn + wn2/(2*g)*(xn2 + yn2);
    A_ = [1 + wn4/g2*xn2,  wn4/g2*xn*yn;
           wn4/g2*xn*yn , 1 + wn4/g2*yn2];
end
NL1_ = diag([NLterm1, NLterm1]);
NL2_ = diag([NLterm2, -NLterm2]);

d_ = -2*zitan*wn;
D_ = [d_, 0; 0, d_];

k_ = - wn2/g*(xdd*sin(psi) + zdd*cos(psi)) - wn2*cos(psi) + psid^2*wn2/g*H;
K_ = [k_ + psid^2, 0; 0, k_];

R_ = [- H*psidd - (xdd*cos(psi) - zdd*sin(psi)) + g*sin(psi);
                              - ydd];

Sd(1:2) = [xnd; ynd];
Sd(3:4) = A_\(D_*[xnd; ynd] + K_*[xn; yn] + R_ + NL1_*[xn; yn] + NL2_*[xnd; ynd]);

end