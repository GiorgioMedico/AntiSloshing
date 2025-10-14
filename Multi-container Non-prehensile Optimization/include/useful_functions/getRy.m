function [R_IB] = getRy(phi)
% GET_RY  Rotation matrix for rotation around y-axis
%	Institute of Robotics
%	Stefan Gadringer, c2019
% 	R_IB = getRy(phi)
% 	R_IB = [I_eB1, I_eB2, I_eB3]
%
%	Input:
%	phi.......Angle in rad
%
%	Output:
%	R_IB........Rotation matrix (SO3)

    R_IB = [cos(phi)  0 sin(phi);...
          0         1 0;...
          -sin(phi) 0 cos(phi)];
end

