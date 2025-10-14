function LB = adBracket(V)

omega = V(1:3);
vel = V(4:6);

% LB = [skew(omega) skew(vel);
%       zeros(3)    skew(omega);];
LB = [skew(omega) zeros(3); 
      skew(vel)   skew(omega);];

end