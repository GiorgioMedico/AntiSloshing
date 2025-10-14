function [quat] = AngleAxis2Quat(theta, n)
% ANGLE_AXIS_2_QUAT  Axis-Angle Representation to Unit Quaternion
%	Institute of Robotics
%	Stefan Gadringer, c2018
% 	quat = AngleAxis2Quat(theta, n)
%
%	Input:
%	theta.......Angle in rad
%	n...........Rotation axis (vector in R3)
%
%	Output:
%	quat........Unit quaternion of rotation

	if(abs(theta) < 1e-10)
		% rotation axis is arbitrary when angle = 0
		quat = [1, 0, 0, 0].';
	else
		quat = [cos(theta/2); sin(theta/2).*n(:)];
	end
	
	% Attention:
	% Same rotation is performed by quat and -quat (as can be seen by
	% computing the rotation matrix out of the quaternion).
	% For uniqueness it is assumed that the scalar part q(1) > 0.
	% Thus, the quaternion has to be multiplied by -1 when q(1) < 0!
  if(quat(1) < 0)
      quat = -quat;
  end
end

