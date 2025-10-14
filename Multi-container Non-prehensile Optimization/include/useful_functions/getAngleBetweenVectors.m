function [angle, same_direction] = getAngleBetweenVectors(v1, v2)
% GET_ANGLE_BETWEEN_TWO_VECTORS  Get angle between two vectors
%	Institute of Robotics
%	Stefan Gadringer, c2019
% 	[angle, same_direction] = getAngleBetweenVectors(v1, v2)
%
%	Input:
%	v1..................Vector 1
%	v2..................Vector 2
%
%	Output:
%	angle...............Angle in rad
%	same_direction......Boolean flag if both vectors show in same direction

    % normalize vectors
    v1 = v1/norm(v1,2);
    v2 = v2/norm(v2,2);

    same_direction = dot(v1,v2) > 0;
    
    % set vector to same direction
    if ~same_direction
        v2 = -v2; %  change direction of the vector
    end
 
    angle =  acos(dot(v1,v2))*180/pi;
end

