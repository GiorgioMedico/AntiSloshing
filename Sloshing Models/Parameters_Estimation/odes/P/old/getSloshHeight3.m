function eta = getSloshHeight3(theta, phi, R)
    x_star = atan(tan(phi)/(tan(theta)*cos(theta)));

    if (-R*tan(theta)*cos(x_star) - R*tan(phi)*sin(x_star)) < 0
        eta = R*tan(theta)*cos(x_star) + R*tan(phi)*sin(x_star);
    else 
        eta = R*tan(theta)*cos(x_star+pi) + R*tan(phi)*sin(x_star+pi)/cos(theta);
    end
end
