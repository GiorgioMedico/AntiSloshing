function [position, velocity, acceleration] = quintic_poly(pos0, posf, vel0, acc0, time)
    % motion_law: Generates a quintic polynomial trajectory
    % Inputs:
    %   pos0: Initial position
    %   posf: Final position
    %   vel0: Initial velocity
    %   acc0: Initial acceleration
    %   time: Time vector (1xn array, typically linspace(0, Te, n))
    % Outputs:
    %   position: Position trajectory
    %   velocity: Velocity trajectory
    %   acceleration: Acceleration trajectory

    % Number of time steps
    n = length(time);

    % Define initial and final conditions
    t0 = time(1); % Start time
    tf = time(end); % End time
    vf = 0; % Final velocity
    af = 0; % Final acceleration

    % Coefficients of quintic polynomial
    % Solving for coefficients of p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    % Using boundary conditions:
    % p(t0) = pos0, p(tf) = posf
    % p'(t0) = vel0, p'(tf) = vf
    % p''(t0) = acc0, p''(tf) = af
    M = [
        1, t0, t0^2,   t0^3,    t0^4,    t0^5;
        0, 1,  2*t0,   3*t0^2,  4*t0^3,  5*t0^4;
        0, 0,  2,      6*t0,    12*t0^2, 20*t0^3;
        1, tf, tf^2,   tf^3,    tf^4,    tf^5;
        0, 1,  2*tf,   3*tf^2,  4*tf^3,  5*tf^4;
        0, 0,  2,      6*tf,    12*tf^2, 20*tf^3;
    ];

    b = [pos0; vel0; acc0; posf; vf; af]; % Boundary conditions
    coeffs = M \ b; % Solve for coefficients [a0; a1; a2; a3; a4; a5]

    % Compute position, velocity, and acceleration
    t = time; % Alias for clarity
    position = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5;
    velocity = coeffs(2) + 2*coeffs(3)*t + 3*coeffs(4)*t.^2 + 4*coeffs(5)*t.^3 + 5*coeffs(6)*t.^4;
    acceleration = 2*coeffs(3) + 6*coeffs(4)*t + 12*coeffs(5)*t.^2 + 20*coeffs(6)*t.^3;
end
