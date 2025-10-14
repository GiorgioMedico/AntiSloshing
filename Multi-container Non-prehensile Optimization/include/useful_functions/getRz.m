function [R_IB] = getRz(phi)
% GET_RZ  Rotation matrix for rotation around z-axis
%	Institute of Robotics
%	Stefan Gadringer, c2019
% 	R_IB = getRz(phi)
% 	R_IB = [I_eB1, I_eB2, I_eB3]
%
%	Input:
%	phi.......Angle in rad
%
%	Output:
%	R_IB........Rotation matrix (SO3)

    R_IB = [cos(phi) -sin(phi) 0;...
          sin(phi) cos(phi)  0;...
          0        0         1];
end

