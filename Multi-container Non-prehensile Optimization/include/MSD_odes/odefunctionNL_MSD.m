function dS = odefunctionNL_MSD(t,S,ks,k,Cs,ms,time,apx,apy,apz,phi_,phip_,phipp_,J,g,as)

  dS = zeros(4,1);

  dS(1,1) = S(3);
  dS(2,1) = S(4);

  ws=sqrt(ks/ms);
  a_vet=[0;0];
  if t>=0 && t<=time(end)
       ax=spline(time,apx,t); %APX E APY SONO DISCRETE NEL DOMINIO DISCRETO TIME, CON SPLINE LE RENDO "CONTINUE" NEL NUOVO DOMINIO DI INTERZIONE T
       ay=spline(time,apy,t);
       az=spline(time,apz,t);
       a_vet(1)=ax;
       a_vet(2)=ay;
       phi=spline(time,phi_,t);
       phip=spline(time,phip_,t);
       phipp=spline(time,phipp_,t);
  else
       a_vet=[0;0];
       az=0;
       phi=0;
       phip=0;
       phipp=0;
  end 

  %FORMULE 3.18a/b CON w=2 (w è COMPRESO TRA 1.5 E 2.5)
  M1=[2*Cs*ws,-2*phip;2*phip,2*Cs*ws]; %Cs è ZITA_S
  A=((ws^4)/g^2)*(S(3).^2+S(4).^2)+(ws^2)*(1+as*(S(1).^2+S(2).^2))+2*Cs*ws*((ws^4)/g^2)*(S(1).*S(3)+S(2).*S(4))-phip.^2+az*(ks/ms)/g;  %HA PRESO W=2, IN S(1).^2 SAREBBE S(1).^2*(W-1)
  %A=(ws^2)-phi.^2;
  M2=[A,-phipp;phipp,A];
  Mrot=[cos(phi) sin(phi);-sin(phi) cos(phi)];
  M3=[1+((ws^4)/g^2)*S(1).^2,((ws^4)/g^2)*S(1).*S(2);((ws^4)/g^2)*S(1).*S(2),1+((ws^4)/g^2)*S(2).^2];
  %M3=[1 0;0 1]
  dS(3:4,1)=M3\(-M1*[S(3);S(4)]-M2*[S(1);S(2)]-Mrot*a_vet);  %E' COME DIRE INV(M3)*...
end