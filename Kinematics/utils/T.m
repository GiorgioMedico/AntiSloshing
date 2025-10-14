function hom=T(R,t)
%T hom=T(R,t)
% generate the 4x4 transformation matrix relative to rotation defined by
% matrix R(3x3) and translation defined by vector t (3x1)
%      |R  t|  
%hom=  |0  1| 
hom=[R t; 0 0 0 1];

end