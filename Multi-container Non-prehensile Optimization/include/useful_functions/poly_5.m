function [s,sd,sdd] = poly_5(s0,s1,sd0,sd1,sdd0,sdd1,t)   

    n = length(t);

    M = [1, 0, 0, 0, 0, 0;
         1, 1*t(end), 1*t(end)^2, 1*t(end)^3, 1*t(end)^4, 1*t(end)^5;
         0, 1, 0, 0, 0, 0;
         0, 1, 2*t(end), 3*t(end)^2, 4*t(end)^3, 5*t(end)^4;
         0, 0, 2, 0, 0, 0;
         0, 0, 2, 6*t(end), 12*t(end)^2, 20*t(end)^3];

    b = [s0;s1;sd0;sd1;sdd0;sdd1];

    A = linsolve(M,b);
    a0 = A(1);
    a1 = A(2);
    a2 = A(3);
    a3 = A(4);
    a4 = A(5);
    a5 = A(6);

    for i=1:n
        s(i) = a0 + a1*t(i) + a2*t(i)^2 + a3*t(i)^3 + a4*t(i)^4 + a5*t(i)^5;
        sd(i) = a1 + 2*a2*t(i) + 3*a3*t(i)^2 + 4*a4*t(i)^3 + 5*a5*t(i)^4;
        sdd(i) = 2*a2 + 6*a3*t(i) + 12*a4*t(i)^2 + 20*a5*t(i)^3;
    end
    
end