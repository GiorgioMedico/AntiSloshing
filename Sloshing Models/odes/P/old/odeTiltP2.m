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

c_phi_y = cos(phi_y);
s_phi_y = sin(phi_y);
c_phi_x = cos(phi_x);
s_phi_x = sin(phi_x);

c_psi = cos(psi);
s_psi = sin(psi);



H = h - Ln;

if strcmp(model_type,'L')
    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3) = Theta_lin(2*mn*wn*zitan,H,g,ln,mn,phi_y,phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,0,0,psid,0,psidd,0,xdd,ydd,zdd);
    Sd(4) = Phi_lin(2*mn*wn*zitan,H,g,ln,mn,phi_y,phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,0,0,psid,0,psidd,0,xdd,ydd,zdd);

elseif strcmp(model_type,'NL')

    Sd(1:2) = [dphi_y; dphi_x];
    Sd(3) = Theta(2*mn*wn*zitan,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,0,0,psid,0,psidd,0,xdd,ydd,zdd);
    Sd(4) = Phi(2*mn*wn*zitan,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,Sd(3),0,psid,0,psidd,0,xdd,ydd,zdd);
    % Sd(3) = Theta(2*mn*zitan,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,1,0,c_psi,s_psi,dphi_y,dphi_x,0,0,0,psid,0,psidd,xdd,ydd,zdd);
    % Sd(4) = Phi(2*mn*zitan,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,1,0,c_psi,s_psi,dphi_y,dphi_x,Sd(3),0,0,psid,0,psidd,xdd,ydd,zdd);
    % Sd(3) = Theta(0,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,0,0,psid,0,psidd,0,xdd,ydd,zdd);
    % Sd(4) = Phi(0,H,g,ln,mn,c_phi_y,s_phi_y,c_phi_x,s_phi_x,c_psi,s_psi,1,0,dphi_y,dphi_x,Sd(3),0,psid,0,psidd,0,xdd,ydd,zdd);
end



end