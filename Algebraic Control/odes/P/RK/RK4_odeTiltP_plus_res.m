function S = RK4_odeTiltP_plus_res(wn,zitan,xdd_0,ydd_0,zdd_0,psi_0,psid_0,psidd_0,dn,g,model_type,time)

n = length(time);

% RK-4 weights
p1 = 1/6;
p2 = 1/3;
p3 = 1/3;
p4 = 1/6;

% RK-4 Integration
S0 = [0;0;0;0];
S = zeros(4,n);
S(:,1) = odeTiltP_PlusTheta_RK4(S0,wn,zitan,xdd_0(1),ydd_0(1),zdd_0(1),psi_0(1),psid_0(1),psidd_0(1),dn,g,model_type);
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

    psi_0_k1 = psi_0(i-1);
    psi_0_k2 = (psi_0(i) + psi_0(i-1))/2;
    psi_0_k3 = psi_0_k2;
    psi_0_k4 = psi_0(i);

    psid_0_k1 = psid_0(i-1);
    psid_0_k2 = (psid_0(i) + psid_0(i-1))/2;
    psid_0_k3 = psid_0_k2;
    psid_0_k4 = psid_0(i);

    psidd_0_k1 = psidd_0(i-1);
    psidd_0_k2 = (psidd_0(i) + psidd_0(i-1))/2;
    psidd_0_k3 = psidd_0_k2;
    psidd_0_k4 = psidd_0(i);

    S_RK_1 = S(:,i-1);
    k1 = odeTiltP_PlusTheta_RK4(S_RK_1,wn,zitan,xdd_0_k1,ydd_0_k1,zdd_0_k1,psi_0_k1,psid_0_k1,psidd_0_k1,dn,g,model_type);

    S_RK_2 = S(:,i-1) + (deltaT*k1/2);
    k2 = odeTiltP_PlusTheta_RK4(S_RK_2,wn,zitan,xdd_0_k2,ydd_0_k2,zdd_0_k2,psi_0_k2,psid_0_k2,psidd_0_k2,dn,g,model_type);

    S_RK_3 = S(:,i-1) + (deltaT*k2/2);
    k3 = odeTiltP_PlusTheta_RK4(S_RK_3,wn,zitan,xdd_0_k3,ydd_0_k3,zdd_0_k3,psi_0_k3,psid_0_k3,psidd_0_k3,dn,g,model_type);

    S_RK_4 = S(:,i-1) + (deltaT*k3);
    k4 = odeTiltP_PlusTheta_RK4(S_RK_4,wn,zitan,xdd_0_k4,ydd_0_k4,zdd_0_k4,psi_0_k4,psid_0_k4,psidd_0_k4,dn,g,model_type);

    S(:,i) = S(:,i-1) + deltaT*(p1*k1 + p2*k2 + p3*k3 + p4*k4);

end


end