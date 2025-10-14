function qd = eom_3d_xy(t,q,u,time,wn,l,zita,alfa,R,app)

ux = u(1,:);        uy = u(2,:);        uz = u(3,:);
x = q(5);           y = q(6);                    
% x2 = x^2;           y2 = y^2;
x2 = q(5)^2;        y2 = q(6)^2;
xd = q(7);          yd = q(8);
% xd2 = xd^2;         yd2 = yd^2;
xd2 = q(7)^2;       yd2 = q(8)^2;

u_x = spline(time,ux,t);
u_y = spline(time,uy,t);
u_z = spline(time,uz,t);

qd = zeros(8,1);
qd(1) = q(3);
qd(2) = q(4);
qd(3) = u_x;
qd(4) = u_y;
qd(5) = q(7);
qd(6) = q(8);


if app

    qd(7) = -u_x - u_z/l*x - 2*zita*wn*xd - wn^2*x;
    qd(8) = -u_y - u_z/l*y - 2*zita*wn*yd - wn^2*y;    
    
else

    rayleigh_x = -2*zita*wn*(xd + 1/(l^2)*(x2*xd + y*yd*x));
    coupling_x = -1/(l^2)*(x*xd2 + x*yd2);
    potential_x = -wn^2*x*(1+alfa/R^2*(x2+y2));

    rayleigh_y = -2*zita*wn*(yd + (y2*yd + x*xd*y)/(l^2));
    coupling_y = -(y*yd2 + y*xd2)/(l^2);
    potential_y = -wn^2*y*(1+alfa/R^2*(x2+y2));
    
    A = [1+x2/l^2,  x*y/l^2;
          x*y/l^2,  1+y2/l^2];
    den = l^2 + x2 + y2;
    A1 = (1/den)*[l^2+y2,  -x*y;
                    -x*y,  l^2+x2];


    qd(7:8) = inv(A)*[rayleigh_x + coupling_x + potential_x - u_x - u_z/l*x;
                      rayleigh_y + coupling_y + potential_y - u_y - u_z/l*y;];
      
end



end