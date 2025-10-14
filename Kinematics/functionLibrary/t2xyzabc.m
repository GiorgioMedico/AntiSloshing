% Creata da Claudio Mazzotti

function XYZABC  = t2xyzabc(T)

ABC = tr2rpy(T,'deg','zyx'); %deg
% ABC = tr2rpy(T,'zyx'); %rad
XYZ = transl(T)'; %mm
XYZABC = [XYZ,ABC];