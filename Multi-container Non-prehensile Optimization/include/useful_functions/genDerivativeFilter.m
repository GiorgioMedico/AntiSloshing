function [ num_der, den_der ] = genDerivativeFilter( f_bound, Ta )

% coefficients of the time continous filter
b = [1 0];
a = [1/(2*pi*f_bound) 1];

% time continous filter
der = tf(b, a);

% time discrete filter
der_d = c2d(der, Ta, 'tustin');

% coefficients of time discrete filter
[num_der, den_der] = tfdata(der_d,'v');

end