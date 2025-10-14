function AdInverseMatrix = Adinv(C)

R = C(1:3,1:3);
r = C(1:3,4);

AdInverseMatrix = [         R'  zeros(3);
                   -R'*skew(r)         R';];

end