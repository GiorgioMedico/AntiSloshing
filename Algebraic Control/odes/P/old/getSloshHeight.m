function eta = getSloshHeight(theta, phi, R)
    x_star = atan2(tan(phi),tan(theta));

    if (-R*tan(theta)*cos(x_star) - R*tan(phi)*sin(x_star)) < 0
        eta = R*tan(theta)*cos(x_star) + R*tan(phi)*sin(x_star);
    else 
        eta = R*tan(theta)*cos(x_star+pi) + R*tan(phi)*sin(x_star+pi);
    end
end

