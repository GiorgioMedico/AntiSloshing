function [x,dx,ddx,dddx]=polgrad5(x0,t,cond)

%cond=1 se acc nulla agli estremi mentre cond=2 se jerk nullo agli estremi
T=t(end);
vmed=x0/T;

x=zeros(1,length(t));
dx=x;
ddx=x;
dddx=x;
if cond==1
A=6*vmed/T^4;
B=-15*vmed/T^3;
C=10*vmed/T^2;

for i=1:length(x)
   x(i)=A*t(i)^5+B*t(i)^4+C*t(i)^3;
   dx(i)=5*A*t(i)^4+4*B*t(i)^3+3*C*t(i)^2;
   ddx(i)=20*A*t(i)^3+12*B*t(i)^2+6*C*t(i);
   dddx(i)=60*A*t(i)^2+24*B*t(i)+6*C;
end
elseif cond==2
    A=vmed/T^4;
    B=-5*vmed/(2*T^3);
    D=5*vmed/(2*T);
    for i=1:length(x)
   x(i)=A*t(i)^5+B*t(i)^4+D*t(i)^2;
   dx(i)=5*A*t(i)^4+4*B*t(i)^3+2*D*t(i);
   ddx(i)=20*A*t(i)^3+12*B*t(i)^2+2*D;
   dddx(i)=60*A*t(i)^2+24*B*t(i);
end
end
end