function C = SE3exp(X,q)

xi = X(1:3);
eta = X(4:6);

R = SO3exp(xi,q);
r = (eye(3)-R)*skew(xi)*eta + (xi'*eta)*q*xi;

C = RotPosToSE3(R,r);

end