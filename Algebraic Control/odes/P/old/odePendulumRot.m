% ODE function for both translational and rotational motions
function dS = odePendulumRot(t,S, time, apx, apy, apz, vphi_c, l, m, g, cs, b, J)
    dS = zeros(6,1);
    
    dS(1,1) = S(4);
    dS(2,1) = S(5);
    dS(3,1) = S(6);

    az = 0;
    if t>=0 && t<=time(end)
       ax=spline(time,apx,t);
       ay=spline(time,apy,t);
       omega_c = spline(time,vphi_c,t);
       %az=spline(time, apz, t);
    else
       ax = 0;
       ay = 0;
       az = 0;
       omega_c = 0;
    end
    
    dS(4,1) = -cs*2*S(4)  -1/l*(g + az)*sin(S(1))/cos(S(2)) - 1/l*cos(S(3))*cos(S(1))/cos(S(2))*ax - 1/l*cos(S(1))*sin(S(3))/cos(S(2))*ay + 2*tan(S(2))*S(4)*S(5) + cos(S(1))*tan(S(2))*b/J*(omega_c - S(6)) + cos(S(1))*sin(S(1))*S(6)^2 + 2*cos(S(1))*S(6)*S(5);
    dS(5,1) = -cs*2*S(5) -1/l*cos(S(2))*cos(S(3))*ay + 1/l*cos(S(2))*sin(S(3))*ax - 1/l*cos(S(1))*sin(S(2))*(g + az) - sin(S(1))*b/J*(omega_c - S(6)) - sin(S(2))*cos(S(2))*S(4)^2 + 1/l*cos(S(3))*sin(S(2))*sin(S(1))*ax + 1/l*sin(S(2))*sin(S(1))*sin(S(3))*ay + cos(S(2))*cos(S(1))^2*sin(S(2))*S(6)^2 - 2*cos(S(2))^2*cos(S(1))*S(6)*S(4); 
    dS(6,1) = b/J*(omega_c - S(6)); %

    %only transl
    % dS(4,1) = -1/l*(g + az)*sin(S(1))/cos(S(2)) - 1/l*cos(S(1))/cos(S(2))*ax + 2*tan(S(2))*S(4)*S(5);
    % dS(5,1) = -1/l*cos(S(2))*ay - 1/l*cos(S(1))*sin(S(2))*(g + az) - sin(S(2))*cos(S(2))*S(4)^2 + 1/l*sin(S(2))*sin(S(1))*ax; 
    
end
