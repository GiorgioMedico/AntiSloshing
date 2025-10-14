function R = Quat2RotMat( quat )
% QUAT_2_ROT_MAT  Unit Quaternion to Rotation Matrix
%	Institute of Robotics
%	Stefan Gadringer, c2018
% 	R = Quat2RotMat(quat)
%
%	Input:
%	quat........Unit quaternion of rotation
%
%	Output:
%	R...........Rotation matrix (SO3)

	qw = quat(1);
	v = quat(2:4);

	R = zeros(3, 3);
	R(1, 1) = 2*(qw^2 + v(1)^2) - 1;
	R(1, 2) = 2*(v(1)*v(2) - qw*v(3));
	R(1, 3) = 2*(v(1)*v(3) + qw*v(2));
	R(2, 1) = 2*(v(1)*v(2) + qw*v(3));
	R(2, 2) = 2*(qw^2 + v(2)^2) - 1;
	R(2, 3) = 2*(v(2)*v(3) - qw*v(1));
	R(3, 1) = 2*(v(1)*v(3) - qw*v(2));
	R(3, 2) = 2*(v(2)*v(3) + qw*v(1));
	R(3, 3) = 2*(qw^2 + v(3)^2) - 1;
end

