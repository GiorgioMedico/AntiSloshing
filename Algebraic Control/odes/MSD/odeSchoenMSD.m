function Sd = odeSchoenMSD(t,S,kn,k,zitan,mn,time,xdd_0,ydd_0,zdd_0,thdd_0,J,g,alpha_n,w,model_type)

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

wn = sqrt(kn/mn);
wn2 = wn^2;
wn4 = wn^4;

g2 = g^2;

Sd = zeros(7,1);

xn    = S(1);
yn    = S(2);
xnd   = S(3);
ynd   = S(4);
thl   = S(5);
thld  = S(6);
thldd = S(7);

xn2 = xn^2;
yn2 = yn^2;
xnd2 = xnd^2;
ynd2 = ynd^2;

d_ = -2*zitan*wn;
D_ = [d_, 2*thld; -2*thld, d_];

k_ = thld^2 - wn2 - zdd*wn2/g;
K_ = [k_, thldd; -thldd, k_];

R_ = [-cos(thl), -sin(thl); sin(thl), -cos(thl)];

if strcmp(model_type,'L')
    NLterm = 0;
    A_ = eye(2);
elseif strcmp(model_type,'NL')
    NLterm = -wn4/g2*(xnd2 + ynd2) - 2*zitan*wn*wn4/g2*(xn*xnd + yn*ynd) - wn2*alpha_n*(xn2 + yn2)^(w-1);
    A_ = [1 + wn4/g2*xn2,  wn4/g2*xn*yn;
           wn4/g2*xn*yn , 1 + wn4/g2*yn2];
end
NL_ = diag([NLterm, NLterm]);

Sd(1:2) = [xnd; ynd];
Sd(3:4) = A_\(D_*[xnd; ynd] + K_*[xn; yn] + R_*[xdd; ydd] + NL_*[xn; yn]);
Sd(5:6) = [thld; thldd];
Sd(7) = k/J*(thdd - thldd);
% Sd(7) = 0.0004/J*(thdd - thldd);

end