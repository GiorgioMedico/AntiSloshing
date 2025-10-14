% Creata da Claudio Mazzotti

% traslazioni in mm
% rotazioni in gradi

function T  = xyzabc2t(XYZABC)

    KUKA_X = XYZABC(1);%mm
    KUKA_Y = XYZABC(2);%mm
    KUKA_Z = XYZABC(3);%mm
    KUKA_A = XYZABC(4);%gradi
    KUKA_B = XYZABC(5);%gradi
    KUKA_C = XYZABC(6);%gradi
    T = transl(KUKA_X,KUKA_Y,KUKA_Z)*trotz(KUKA_A,'deg')*troty(KUKA_B,'deg')*trotx(KUKA_C,'deg');
    