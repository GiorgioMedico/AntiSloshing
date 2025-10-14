function [robot] = create_SmartSix()
    robot.a = [0.15, 0.59, 0.13, 0, 0, 0];
    robot.d = [0.45, 0, 0, 0.6471, 0, 0.095];
    robot.alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];

    robot.q_min = [-2.8798, -1.4835, -2.9671, -3.6652, -2.3562, -47.1239];
    robot.q_max = [2.8798, 2.7053, 0, 3.6652, 2.3562, 47.1239];

    robot.qp_min = [-2.44346, -2.79253, -2.96706, -7.85398, -6.54498, -9.59931];
    robot.qp_max = - robot.qp_min;

    robot.q_c0 = [0.0; pi/2; -pi/2; 0.0; 0.0; 0.0];
    robot.q_dh0 = [0.0; -pi/2; -pi/2; 0.0; 0.0; 0.0];
    robot.S = diag([-1,1,-1,-1,1,-1]);
 
end