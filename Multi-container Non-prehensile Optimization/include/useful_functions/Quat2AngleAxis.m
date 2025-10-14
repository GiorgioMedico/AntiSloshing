function [theta, n] = Quat2AngleAxis(quat)
% QUAT_2_ANGLE_AXIS  Unit Quaternion to Axis-Angle Representation
%	Institute of Robotics
%	Stefan Gadringer, c2018
% 	[theta, n] = Quat2AngleAxis(quat)
%
%	Input:
%	quat........Unit quaternion of rotation
%
%	Output:
%	theta.......Angle in rad
%	n...........Rotation axis (vector in R3)

	% angle is always positive
	theta = 2*acos(min([1, max([-1, quat(1)])]));

	if(abs(theta) < 1e-10)
		% rotation axis is arbitrary when angle = 0
		n = [1, 0, 0].';
		theta = 0;
	else
		% compute rotation axis
		tmp = quat(2:4);
		n = tmp(:)./sin(theta/2);
		
		% normalize (although this might not be neccessary)
		n = n./norm(n, 2);
	end
end

