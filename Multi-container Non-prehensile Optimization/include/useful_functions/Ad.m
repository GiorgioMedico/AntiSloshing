function AdMatrix = Ad(C)

R = C(1:3,1:3);
r = C(1:3,4);

AdMatrix = [        R  zeros(3);
            skew(r)*R         R;];

end