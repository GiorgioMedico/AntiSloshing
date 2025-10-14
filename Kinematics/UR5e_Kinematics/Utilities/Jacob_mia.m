function Jac = Jacob_mia(q,robot)
%q è il valore istantaneo delle posizioni dei giunti

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

R = [R00,R01,R02,R03,R04,R05];
    
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

Tb = [T00,T01,T02,T03,T04,T05];

Jac = [skew(R00(:,3))*(T06(1:3,4) - T00(1:3,4)), skew(R01(:,3))*(T06(1:3,4) - T01(1:3,4)), skew(R02(:,3))*(T06(1:3,4) - T02(1:3,4)), skew(R03(:,3))*(T06(1:3,4) - T03(1:3,4)), skew(R04(:,3))*(T06(1:3,4) - T04(1:3,4)),skew(R05(:,3))*(T06(1:3,4) - T05(1:3,4));
        R00(:,3), R01(:,3), R02(:,3), R03(:,3), R04(:,3), R05(:,3) ];

% Jac = zeros(6,6);
% for j = 1:6
%     Jac(1:6,j) = [skew(R(:,3*j))*(T06(1:3,4) - Tb(1:3,4*j));
%             R(:,3*j)];
% end

end
