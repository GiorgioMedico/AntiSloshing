function [R_IB] = getRx(phi)
% GET_RX  Rotation matrix for rotation around x-axis
%	Institute of Robotics
%	Stefan Gadringer, c2019
% 	R_IB = getRx(phi)
% 	R_IB = [I_eB1, I_eB2, I_eB3]
%
%	Input:
%	phi.......Angle in rad
%
%	Output:
%	R_IB........Rotation matrix (SO3)

    R_IB = [1 0        0;...
          0 cos(phi) -sin(phi);...
          0 sin(phi) cos(phi)];
end

