function err=media_errore(Altezza_conf,Altezza_rif,time_conf,time_rif,time_fin)
parameter=1;
while time_rif(parameter)<time_fin
    parameter=parameter+1;
end

HL1m=zeros(1,parameter);
for i=1:parameter
    HL1m(i)=spline(time_conf,Altezza_conf,time_rif(i));
end

sum=0;
for j=1:parameter
    delta=abs(HL1m(j)-Altezza_rif(j));
    sum=sum+delta;
end

% err=sum/parameter;
err=100*sum/parameter/max(Altezza_rif(1:parameter));
end