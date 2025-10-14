function A = RotPosToSE3(Rot,r)

A = [Rot        r;
     zeros(1,3) 1;];

end