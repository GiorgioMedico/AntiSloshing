function [ A ] = Quat2Drehmat( alpha )
%#codegen

eta = alpha(1);
eps = alpha(2:4);

A = zeros(3, 3);
A(1, 1) = 2*(eta^2 + eps(1)^2) - 1;
A(1, 2) = 2*(eps(1)*eps(2) - eta*eps(3));
A(1, 3) = 2*(eps(1)*eps(3) + eta*eps(2));
A(2, 1) = 2*(eps(1)*eps(2) + eta*eps(3));
A(2, 2) = 2*(eta^2 + eps(2)^2) - 1;
A(2, 3) = 2*(eps(2)*eps(3) - eta*eps(1));
A(3, 1) = 2*(eps(1)*eps(3) - eta*eps(2));
A(3, 2) = 2*(eps(2)*eps(3) + eta*eps(1));
A(3, 3) = 2*(eta^2 + eps(3)^2) - 1;

end

