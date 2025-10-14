function eta = getSloshHeight2(theta, phi, R)
    eta = R*sqrt(tan(theta)^2 + (tan(phi)^2)/(cos(theta)^2));
end
