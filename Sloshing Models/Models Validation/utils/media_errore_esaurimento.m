function err = media_errore_esaurimento(Altezza_conf, Altezza_rif, time_conf, time_rif, time_fin)
% Find the start index from which we begin calculating the error
parameter_start = 1;
while time_rif(parameter_start) < time_fin
    parameter_start = parameter_start + 1;
end

% Find the end index of time_rif
parameter_end = length(time_rif);

% Calculate the length of the range
num_points = parameter_end - parameter_start + 1;

% Initialize the interpolated heights array
HL1m = zeros(1, num_points);

% Interpolate Altezza_conf at time_rif points from time_fin to the end
for i = parameter_start:parameter_end
    HL1m(i - parameter_start + 1) = spline(time_conf, Altezza_conf, time_rif(i));
end

% Compute the error sum
sum_error = 0;
for j = 1:num_points
    delta = abs(HL1m(j) - Altezza_rif(parameter_start + j - 1));
    sum_error = sum_error + delta;
end

% Calculate the average error
% err = sum_error / num_points;
err = 100*sum_error/num_points/max(Altezza_rif((parameter_start-1)+(1:num_points)));
end

