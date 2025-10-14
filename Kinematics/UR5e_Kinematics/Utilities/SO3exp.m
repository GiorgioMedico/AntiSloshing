function R = SO3exp(xi,phi)

if isempty(phi)
    phi = norm(xi);
    e = xi/phi;
else
    e = xi;
end

R = eye(3) + sin(phi)*skew(e) + (1-cos(phi))*skew(e)*skew(e);

end