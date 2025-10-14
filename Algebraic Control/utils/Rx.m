function Rot_x=Rx(ang)
% Rot_x= Rx(angle)
% create the matrix related to rotation (angle) about axis x
%
%       |1         0           0     |  
% Rot_x=|0     cos(angle) -sin(angle)|
%       |0     sin(angle)  cos(angle)|

Rot_x=[1 0 0;0 cos(ang) -sin(ang);0 sin(ang) cos(ang);];

end