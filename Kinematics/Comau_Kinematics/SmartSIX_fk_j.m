%% DH transformation matrices and direct kinematics of a serial robot (SCARA example)

clear all
close all
clc

%% TOOL OFFSET
T_ee_tcp = [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1;];

%% Define symbolic variables
syms alpha d a theta

%% number of joints of COMAU SMARTSIX
N=6;

%% Insert DH table of parameters of COMAU SMARTSIX

% %% DH table of numerical parameters of COMAU
syms a1 a2 a3 
syms d1 d4 d6
syms q1 q2 q3 q4 q5 q6

% actual comau
% DHTABLE = [ -pi/2    a1   d1    q1;
%              pi      a2    0    q2;
%              pi/2    a3    0    q3;
%             -pi/2     0   d4    q4;
%              pi/2     0    0    q5;
%               0       0   d6    q6];

% dh comau 
DHTABLE = [ -pi/2    a1   d1    q1;
             0       a2    0    q2;
            -pi/2    a3    0    q3;
             pi/2     0   d4    q4;
            -pi/2     0    0    q5;
              0       0   d6    q6];  

q_c0 = [0.0; pi/2; -pi/2; 0.0; 0.0; 0.0];
q_dh0 = [0.0; -pi/2; -pi/2; 0.0; 0.0; 0.0];
S = diag([-1,1,-1,-1,1,-1]);

%% Build the general Denavit-Hartenberg trasformation matrix
TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrices for each link
% First, we create an empty cell array
A = cell(1,N); % hom matrix from joint i to joint i-1
Ai_0 = cell(1,N); % hom matrix from joint i to joint 0

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix
% for i = 1:N
%     alpha = DHTABLE(i,1);
%     a = DHTABLE(i,2);
%     d = DHTABLE(i,3);
%     theta = DHTABLE(i,4);
% 
%     A{i} = subs(TDH);
% end

for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = q_dh0(i) + S(i,i)*DHTABLE(i,4);
    
    A{i} = subs(TDH);
end

%% Direct kinematics
T = eye(4);
for i=1:N 
    T = T*A{i};
    T = simplify(T);
    Ai_0{i} = T; % Store T in the tensor
end
% output TN matrix
T0N = T;

%% Definition of the Forward kinematics function and of the matalb functions 
% Kinematic Parameters
a1 = 0.15;
a2 = 0.59;
a3 = 0.13;
d1 = 0.45;
d4 = 0.6471;
d6 = 0.095;

syms f
q = [q1 q2 q3 q4 q5 q6];

%% Geometric
position = T0N(1:3, 4);
% Extract the rotation matrix part (upper-left 3x3 submatrix)
rotation_matrix = T0N(1:3, 1:3);

% Initialize the Jacobian matrix
Jg = sym(zeros(6, 6));

% Compute the Jacobian matrix
for i = 1:6
    if i == 1
        % The base frame's z-axis
        z_i = [0; 0; 1];
        p_i = [0; 0; 0];
    else
        % Extract the rotation matrix from the base to joint i-1
        R_i = Ai_0{i-1}(1:3, 1:3);
        z_i = R_i(:, 3);  % z-axis of the (i-1)th frame is the rotation axis for joint i
        p_i = Ai_0{i-1}(1:3, 4);
    end
    % Compute the linear velocity part of the Jacobian (cross product)
    Jv = cross(z_i, (position - p_i));

    % Compute the angular velocity part of the Jacobian
    Jw = z_i;

    % Assign the columns of the Jacobian matrix
    Jg(1:3, i) = Jv;
    Jg(4:6, i) = Jw;
end

Jg = Jg*S;
% Display the geometric Jacobian matrix
fprintf('Jg = \n\n');
disp(Jg);

f = T0N;

%% Matlab functions

matlabFunction(subs(f), 'file', 'SmartSix_FK','vars', {[q1; q2; q3; q4; q5; q6]},'outputs',{'x'});
% matlabFunction(subs(Ja), 'file', 'COMAU_Ja','vars', {[q1; q2; q3; q4; q5; q6]},'outputs',{'J'});
matlabFunction(subs(Jg), 'file', 'SmartSix_Jg','vars', {[q1; q2; q3; q4; q5; q6]},'outputs',{'J'});

%%
q_c = [0.4, 0.2, -1.6, 0.2, -1.6, -0.4]';
q_c = [0, 0, 0, 0, 0, -0]';

SmartSix_FK(q_c)

q_dh_0 = -S*q_c0;

% q_dh = q_dh0 + S*q_c;

% COMAU_FK(q_c)
% T06 = SmartSix_FK(q_dh)




% Tb0 = [-1 0 0 0;
%        0 1 0 0;
%        0 0 -1 0;
%        0 0 0 1;];
% Tb6 = Tb0*T06

% J = COMAU_Jg([0; 0; -pi/2; 0; -pi/2; 0])
% pJ = pinv(J)
