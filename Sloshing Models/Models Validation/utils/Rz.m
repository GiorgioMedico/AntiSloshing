function Rot_z=Rz(ang)
% Rot_z= Rz(angle)
% Create the matrix related to rotation (angle) about axis z
%
%       | cos(angle)   -sin(angle)  0|  
% Rot_z=| sin(angle)    cos(angle)  0|       
%       |     0            0        1|   
Rot_z=[cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1];
end