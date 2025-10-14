% Creata da Claudio Mazzotti

function XYZABC  = t2xyzBCA(T)

XYZ = transl(T)'; %mm

% roty(B)*rotx(C)*rotz(A)
% [ cos(A)*cos(B) + sin(A)*sin(B)*sin(C), cos(A)*sin(B)*sin(C) - cos(B)*sin(A), cos(C)*sin(B)]
% [                        cos(C)*sin(A),                        cos(A)*cos(C),       -sin(C)]
% [ cos(B)*sin(A)*sin(C) - cos(A)*sin(B), sin(A)*sin(B) + cos(A)*cos(B)*sin(C), cos(B)*cos(C)]

% % C = (0,pi)
% B = atan2d(T(1,3),T(3,3));
% C = atan2d(sqrt((T(1,3)^2 + T(3,3)^2)),-T(2,3));
% A = atan2d(T(2,1),T(2,2));

ABC = tr2eul(T,'deg','flip'); %deg
XYZABC = [XYZ,ABC];%mm e gradi