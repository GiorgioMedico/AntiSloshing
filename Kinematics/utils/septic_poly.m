function [position, velocity, acceleration, jerk] = septic_poly(pos0, posf, vel0, acc0, jerk0, time)
    % SEPTIC_POLY: Generates a septic (7th-degree) polynomial trajectory with C³ continuity
    % Inputs:
    %   pos0: Initial position
    %   posf: Final position
    %   vel0: Initial velocity
    %   acc0: Initial acceleration
    %   jerk0: Initial jerk
    %   time: Time vector (1xn array, typically linspace(0, Te, n))
    % Outputs:
    %   position: Position trajectory
    %   velocity: Velocity trajectory
    %   acceleration: Acceleration trajectory
    %   jerk: Jerk trajectory

    % Number of time steps
    n = length(time);

    % Define initial and final conditions
    t0 = time(1); % Start time
    tf = time(end); % End time
    vf = 0;  % Final velocity
    af = 0;  % Final acceleration
    jerkf = 0; % Final jerk

    % Coefficients of septic polynomial (7th-degree)
    % p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
    % Using boundary conditions:
    % p(t0) = pos0,  p(tf) = posf
    % p'(t0) = vel0, p'(tf) = vf
    % p''(t0) = acc0, p''(tf) = af
    % p'''(t0) = jerk0, p'''(tf) = jerkf
    M = [
        1, t0, t0^2,   t0^3,    t0^4,     t0^5,     t0^6,      t0^7;
        0, 1,  2*t0,   3*t0^2,  4*t0^3,   5*t0^4,   6*t0^5,    7*t0^6;
        0, 0,  2,      6*t0,    12*t0^2,  20*t0^3,  30*t0^4,   42*t0^5;
        0, 0,  0,      6,       24*t0,    60*t0^2,  120*t0^3,  210*t0^4;
        1, tf, tf^2,   tf^3,    tf^4,     tf^5,     tf^6,      tf^7;
        0, 1,  2*tf,   3*tf^2,  4*tf^3,   5*tf^4,   6*tf^5,    7*tf^6;
        0, 0,  2,      6*tf,    12*tf^2,  20*tf^3,  30*tf^4,   42*tf^5;
        0, 0,  0,      6,       24*tf,    60*tf^2,  120*tf^3,  210*tf^4;
    ];

    b = [pos0; vel0; acc0; jerk0; posf; vf; af; jerkf]; % Boundary conditions
    coeffs = M \ b; % Solve for coefficients [a0; a1; a2; a3; a4; a5; a6; a7]

    % Compute position, velocity, acceleration, and jerk
    t = time; % Alias for clarity
    position = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5 + coeffs(7)*t.^6 + coeffs(8)*t.^7;
    velocity = coeffs(2) + 2*coeffs(3)*t + 3*coeffs(4)*t.^2 + 4*coeffs(5)*t.^3 + 5*coeffs(6)*t.^4 + 6*coeffs(7)*t.^5 + 7*coeffs(8)*t.^6;
    acceleration = 2*coeffs(3) + 6*coeffs(4)*t + 12*coeffs(5)*t.^2 + 20*coeffs(6)*t.^3 + 30*coeffs(7)*t.^4 + 42*coeffs(8)*t.^5;
    jerk = 6*coeffs(4) + 24*coeffs(5)*t + 60*coeffs(6)*t.^2 + 120*coeffs(7)*t.^3 + 210*coeffs(8)*t.^4;
end
