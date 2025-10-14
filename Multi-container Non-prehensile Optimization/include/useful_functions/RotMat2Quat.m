function [quat] = RotMat2Quat(R)
% ROT_MAT_2_QUAT  Rotation Matrix to Unit Quaternion
%	Institute of Robotics
%	Stefan Gadringer, c2018
% 	quat = RotMat2Quat(R)
%
%	Input:
%	R...........Rotation matrix (SO3)
%
%	Output:
%	quat........Unit quaternion of rotation

  quat = zeros(4,1);
  
  tol = eps; % tolerance

  % calculate argument in the sqare root of all 4 cases
  sqrt_arg_vec = zeros(4,1);
  sqrt_arg_vec(1) = 1 + R(1, 1) + R(2, 2) + R(3, 3);
  sqrt_arg_vec(2) = 1 + R(1, 1) - R(2, 2) - R(3, 3);
  sqrt_arg_vec(3) = 1 - R(1, 1) + R(2, 2) - R(3, 3);
  sqrt_arg_vec(4) = 1 - R(1, 1) - R(2, 2) + R(3, 3);

  % get index of largest argument in the sqare root
  [sqrt_arg,index] = max(sqrt_arg_vec);
  
  root = sqrt(sqrt_arg); % use positive square root
  temp = 1/(2 * root);
  
  switch index
    case 1
%       disp(1);
      quat(1) = 0.5 * root;
      quat(2) = (R(3, 2) - R(2, 3)) * temp;
      quat(3) = (R(1, 3) - R(3, 1)) * temp;
      quat(4) = (R(2, 1) - R(1, 2)) * temp;
    case 2
%       disp(2)
      quat(1) = (R(3, 2) - R(2, 3)) * temp;
      quat(2) = 0.5 * root;
      quat(3) = (R(1, 2) + R(2, 1)) * temp;
      quat(4) = (R(1, 3) + R(3, 1)) * temp;
    case 3
%       disp(3)
      quat(1) = (R(1, 3) - R(3, 1)) * temp;
      quat(2) = (R(1, 2) + R(2, 1)) * temp;
      quat(3) = 0.5 * root;
      quat(4) = (R(2, 3) + R(3, 2)) * temp;
    case 4
%       disp(4)
      quat(1) = (R(2, 1) - R(1, 2)) * temp;
      quat(2) = (R(1, 3) + R(3, 1)) * temp;
      quat(3) = (R(2, 3) + R(3, 2)) * temp;
      quat(4) = 0.5 * root;
    otherwise
        disp('other value')
  end

  % normalize the unit quaternion to be sure
  quat = quat / norm(quat);
  
  % check if norm of the vector part is very small
  if norm(quat(2:4)) < tol
    quat = [1; 0; 0; 0];
  end
  
  % Attention:
  % Same rotation is performed by q and -q (as can be seen by
  % computing the rotation matrix out of the quaternion).
  % For uniqueness it is assumed that the scalar part q(1) > 0.
  % Thus, the quaternion has to be multiplied by -1 when q(1) < 0!
  if(quat(1) < 0)
      quat = -quat;
  end
end

