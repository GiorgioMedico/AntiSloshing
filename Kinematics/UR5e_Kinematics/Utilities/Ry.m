function Rot_y=Ry(ang)
% Rot_y= Ry(angle)
% Create the matrix related to rotation (angle) about axis y
%
%       | cos(angle)   0    sin(angle)|  
% Rot_y=|    0         1        0     |
%       |-sin(angle)   0    cos(angle)|


Rot_y=[cos(ang) 0 sin(ang);0 1 0;-sin(ang) 0  cos(ang);];
end