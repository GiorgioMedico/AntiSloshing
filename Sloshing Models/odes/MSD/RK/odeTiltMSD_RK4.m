function Sd = odeTiltMSD_RK4(S,kn,zitan,mn,xdd_0,ydd_0,zdd_0,psi_0,psid_0,psidd_0,h,hn,g,alpha_n,w,model_type)

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
    NLterm1 = -wn4/g2*(xnd2 + ynd2) - 2*zitan*wn*wn4/g2*(xn*xnd + yn*ynd) - wn2*alpha_n*(xn2 + yn2)^(w-1) + psidd_0*wn2/g*xn ;
    NLterm2 = -2*psid_0*wn2/g*yn;
    H = h/2 + hn + wn2/(2*g)*(xn2 + yn2);
    A_ = [1 + wn4/g2*xn2,  wn4/g2*xn*yn;
           wn4/g2*xn*yn , 1 + wn4/g2*yn2];
end
NL1_ = diag([NLterm1, NLterm1]);
NL2_ = diag([NLterm2, -NLterm2]);

d_ = -2*zitan*wn;
D_ = [d_, 0; 0, d_];

k_ = - wn2/g*(xdd_0*sin(psi_0) + zdd_0*cos(psi_0)) - wn2*cos(psi_0) + psid_0^2*wn2/g*H;
K_ = [k_ + psid_0^2, 0; 0, k_];

R_ = [- H*psidd_0 - (xdd_0*cos(psi_0) - zdd_0*sin(psi_0)) + g*sin(psi_0);
                              - ydd_0];

Sd(1:2) = [xnd; ynd];
Sd(3:4) = A_\(D_*[xnd; ynd] + K_*[xn; yn] + R_ + NL1_*[xn; yn] + NL2_*[xnd; ynd]);

end