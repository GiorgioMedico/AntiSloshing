function S = RK4_odeSchoenP_res(wn,zitan,xdd_0,ydd_0,zdd_0,th_0,thd_0,thdd_0,g,model_type,time)

n = length(time);

% RK-4 weights
p1 = 1/6;
p2 = 1/3;
p3 = 1/3;
p4 = 1/6;

% RK-4 Integration
S0 = [0;0;0;0];
S = zeros(4,n);
S(:,1) = odeSchoenP_RK4(S0,wn,zitan,xdd_0(1),ydd_0(1),zdd_0(1),th_0(1),thd_0(1),thdd_0(1),g,model_type);
for i = 2:n

    deltaT = time(i) - time(i-1);

    xdd_0_k1 = xdd_0(i-1);
    xdd_0_k2 = (xdd_0(i) + xdd_0(i-1))/2;
    xdd_0_k3 = xdd_0_k2;
    xdd_0_k4 = xdd_0(i);

    ydd_0_k1 = ydd_0(i-1);
    ydd_0_k2 = (ydd_0(i) + ydd_0(i-1))/2;
    ydd_0_k3 = ydd_0_k2;
    ydd_0_k4 = ydd_0(i);

    zdd_0_k1 = zdd_0(i-1);
    zdd_0_k2 = (zdd_0(i) + zdd_0(i-1))/2;
    zdd_0_k3 = zdd_0_k2;
    zdd_0_k4 = zdd_0(i);

    th_0_k1 = th_0(i-1);
    th_0_k2 = (th_0(i) + th_0(i-1))/2;
    th_0_k3 = th_0_k2;
    th_0_k4 = th_0(i);

    thd_0_k1 = thd_0(i-1);
    thd_0_k2 = (thd_0(i) + thd_0(i-1))/2;
    thd_0_k3 = thd_0_k2;
    thd_0_k4 = thd_0(i);

    thdd_0_k1 = thdd_0(i-1);
    thdd_0_k2 = (thdd_0(i) + thdd_0(i-1))/2;
    thdd_0_k3 = thdd_0_k2;
    thdd_0_k4 = thdd_0(i);

    S_RK_1 = S(:,i-1);
    k1 = odeSchoenP_RK4(S_RK_1,wn,zitan,xdd_0_k1,ydd_0_k1,zdd_0_k1,th_0_k1,thd_0_k1,thdd_0_k1,g,model_type);

    S_RK_2 = S(:,i-1) + (deltaT*k1/2);
    k2 = odeSchoenP_RK4(S_RK_2,wn,zitan,xdd_0_k2,ydd_0_k2,zdd_0_k2,th_0_k2,thd_0_k2,thdd_0_k2,g,model_type);

    S_RK_3 = S(:,i-1) + (deltaT*k2/2);
    k3 = odeSchoenP_RK4(S_RK_3,wn,zitan,xdd_0_k3,ydd_0_k3,zdd_0_k3,th_0_k3,thd_0_k3,thdd_0_k3,g,model_type);

    S_RK_4 = S(:,i-1) + (deltaT*k3);
    k4 = odeSchoenP_RK4(S_RK_4,wn,zitan,xdd_0_k4,ydd_0_k4,zdd_0_k4,th_0_k4,thd_0_k4,thdd_0_k4,g,model_type);

    S(:,i) = S(:,i-1) + deltaT*(p1*k1 + p2*k2 + p3*k3 + p4*k4);

end


end