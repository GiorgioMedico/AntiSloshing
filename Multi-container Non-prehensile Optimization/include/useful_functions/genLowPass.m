function [ num_lp, den_lp, fact_lp ] = genLowPass( f_bound, Ta )

% coefficients of the time continous filter
b = [1];
a = [1/(2*pi*f_bound) 1];

% generation of filter
tp = tf(b, a);

% time discrete filter
tp_d = c2d(tp, Ta, 'tustin');

% initial conditions for filter (zero output) and coefficients
[num_lp, den_lp] = tfdata(tp_d, 'v');
b1 = num_lp(1);
b2 = num_lp(2);
a1 = den_lp(1);
a2 = den_lp(2);
fact_lp = (1 - b1/a1)/(b2 - b1*a2/a1);

% fact_lp describes:
% q0_lp = fact_lp*q0;

end