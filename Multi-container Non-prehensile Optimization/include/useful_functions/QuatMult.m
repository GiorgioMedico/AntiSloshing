function q_res = QuatMult(q1, q2)
% QUAT_MULT  Composition of the rotations R1*R2 represented by Unit Quaternions
%	Institute of Robotics
% 	q_res = QuatMult(q1, q2)
%
%	Input:
%	q1..........Unit quaternion of rotation 1
%	q2..........Unit quaternion of rotation 2
%
%	Output:
%	q_res.......Unit quaternion of composed rotation (rotation matrix would be R = R1*R2)

  q1 = q1(:);   % convert to column vector
  q2 = q2(:);   % convert to column vector

  % compute the composition of the rotations
  q_res = zeros(4, 1);
  q_res(1) = q1(1)*q2(1) - q1(2:4).'*q2(2:4);
  q_res(2:4) = q1(1)*q2(2:4) + q2(1)*q1(2:4) + SkewMat33(q1(2:4))*q2(2:4);

  % Attention:
  % Same rotation is performed by q_res and -q_res (as can be seen by
  % computing the rotation matrix out of the quaternion).
  % For uniqueness it is assumed that the scalar part q_res(1) > 0.
  % Thus, the quaternion has to be multiplied by -1 if q_res(1) < 0!
  if(q_res(1) < 0)
      q_res = -q_res;
  end

end
