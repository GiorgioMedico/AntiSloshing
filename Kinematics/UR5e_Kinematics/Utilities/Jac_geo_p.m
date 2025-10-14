function Jg_p = Jac_geo_p(q,qp,robot)

R00 = eye(3);
R01 = Rx(pi/2)*Ry(q(1));
R12 = Rz(q(2));
R23 = Rz(q(3));
R34 = Rx(pi/2)*Ry(q(4));
%R45 = Rx(-pi/2)*Ry(q(5));
R45 = Rz(q(5))*Rx(-pi/2);
R56 = Rz(q(6));

R02 = R01*R12;
R03 = R02*R23;
R04 = R03*R34;
R05 = R04*R45;
R06 = R05*R56;

R01p  = [-sin(q(1)) 0 cos(q(1)); cos(q(1)) 0 sin(q(1)); 0 0 0;]*qp(1);
R12p = [-sin(q(2)) -cos(q(2)) 0; cos(q(2)) -sin(q(2)) 0; 0 0 0;]*qp(2);
R23p = [-sin(q(3)) -cos(q(3)) 0; cos(q(3)) -sin(q(3)) 0; 0 0 0;]*qp(3);
R34p = [-sin(q(4)) 0 cos(q(4)); cos(q(4)) 0 sin(q(4)); 0 0 0;]*qp(4);
R45p = [-sin(q(5)) 0 -cos(q(5)); cos(q(5)) 0 -sin(q(5)); 0 0 0;]*qp(5);
R56p = [-sin(q(6)) -cos(q(6)) 0; cos(q(6)) -sin(q(6)) 0; 0 0 0;]*qp(6);

T00 = HT(R00,[0,0,0]);
T01 = HT(R01,[0,0,robot.d(1)]);
T12 = HT(R12,(R12*[-robot.a(2),0,0]')');   
T23 = HT(R23,(R23*[-robot.a(3),0,0]')');
T34 = HT(R34,[0,0,robot.d(4)]);
T45 = HT(R45,[0,0,robot.d(5)]);
T56 = HT(R56,[0,0,robot.d(6)]);

T02 = T01*T12;
T03 = T02*T23; %varia 3 mm
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

T01p = [R01p [0;0;0]; 0,0,0,0];
T12p = [R12p R12p*[-robot.a(2),0,0]'; 0,0,0,0];
T23p = [R23p R23p*[-robot.a(3),0,0]'; 0,0,0,0];
T34p = [R34p [0;0;0]; 0,0,0,0];
T45p = [R45p [0;0;0]; 0,0,0,0];
T56p = [R56p [0;0;0]; 0,0,0,0];

Ra = [R01, R12, R23, R34, R45, R56];
Rb = [R01,R02,R03,R04,R05];
Ta = [T01, T12, T23, T34, T45, T56];
Tb = [T01,T02,T03,T04,T05];

Rp = [R01p, R12p, R23p, R34p, R45p, R56p];
Tp = [T01p, T12p, T23p, T34p, T45p, T56p];

%% JOp
%zp = zeros(3,5);
zp = casadi.SX.sym('zp',3,5);
for i = 1:5
   %Rm = zeros(3*i,15);
   Rm = casadi.SX.sym('Rm',3*i,15);
    for j = 1:i
        Rm(3*j-2:3*j,:) = [eye(3),eye(3),eye(3),eye(3),eye(3)]; %riempo con matrici identiche
    end
    zpf = [0 0 0]';
    for j = 1:i
          Rm(3*j-2:3*j,1:3*i) = Ra(:,1:3*i);  %riempo tutte le righe
    end
    for j = 1:i
        Rm(3*j-2:3*j,3*j-2:3*j) = Rp(:,3*j-2:3*j);   %riempo solo la diagonale
    end
   
    for k = 1:i
        Rf = Rm(3*k-2:3*k,1:3);  
        for j = 2:5
            Rf = Rf*Rm(3*k-2:3*k,3*j-2:3*j);  %eseguo moltiplicazioni
        end
        zpf = Rf*[0 0 1]' + zpf;  %sommo ogni riga
    end
    zp(:,i) = zpf;
end

%% JPp
%pp = zeros(3,5);
pp = casadi.SX.sym('pp',3,5);
for i=1:5
    %Tm = zeros(4*i,20);
    Tm = casadi.SX.sym('Tm',4*i,20);
    for j = 1:i
        Tm(4*j-3:4*j,:) = [eye(4),eye(4),eye(4),eye(4),eye(4)]; %riempo con matrici identiche
    end
    ppf = [0 0 0 0]';
    for j=1:i
        Tm(4*j-3:4*j,1:4*i) = [Ta(:,1:4*i)];
    end
    for j = 1:i
        Tm(4*j-3:4*j,4*j-3:4*j) = Tp(:,4*j-3:4*j);
    end

    for k = 1:i
        Tf = Tm(4*k-3:4*k,1:4);
        for j = 2:5
            Tf = Tf*Tm(4*k-3:4*k,4*j-3:4*j);
        end
        ppf = Tf*[0 0 0 1]' + ppf;
    end
    pp(:,i) = ppf(1:3,:);
end

%jpp = zeros(3,5);
jpp = casadi.SX.sym('jpp',3,5);
for i = 1:5
    jpp(:,i) = skew(zp(:,i))*(T06(1:3,4)-Tb(1:3,4*i)) + skew(Rb(:,3*i))*(pp(:,5)-pp(:,i));
end

%% Jg_p
%Jg_p = zeros(6,6);
Jg_p = casadi.SX.sym('Jg_p',6,6);
jpp0 = skew([0 0 1]')*pp(:,5);
Jg_p(:,1) = [jpp0; [0 0 0]'];
for i=1:5
    Jg_p(:,i+1) = [jpp(:,i); zp(:,i)];
end

end
