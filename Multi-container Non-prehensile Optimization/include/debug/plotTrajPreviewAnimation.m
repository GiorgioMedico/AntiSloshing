numSteps = 1000;
N_eval = 1000;
lut_map = get_p_07_0.map(N_eval); % maps the parametric values to the B-spline values
T06_map = get_T06.map(N_eval);
par_vec = linspace(0, 1, N_eval); % Parametric values ranging from 0 to 1, evenly spaced
val_mat = full(lut_map(par_vec));
T06_val = full(T06_map(par_vec));

jointTrajectory = zeros(6,N_eval);
for i=1:N_eval
    q_eval = SmartSix_IK(robot,T06_val(:,4*(i-1)+1 : 4*i),0);
    jointTrajectory(:,i) = [q_eval(1), q_eval(2), q_eval(3), q_eval(4), q_eval(5), q_eval(6)]';
end

fg = figure;
set(fg, 'Name', 'Robot Joint Trajectory Preview', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);

fg.WindowState = 'maximized';
plot3(val_mat(1,:),val_mat(2,:),val_mat(3,:),'LineWidth',2)
hold on;
axis equal;
view(3);
title('Robot Joint Trajectory Preview','Interpreter','latex');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
% Add lighting to the figure
light('Position', [1 0 1], 'Style', 'infinite');
% light('Position', [-1 -1 1], 'Style', 'infinite');

% Optionally, add a light behind the robot for balanced lighting
light('Position', [0 0 -1], 'Style', 'infinite');

disp('Press any key in the plot window to start the animation...');
waitforbuttonpress;

% Set up rate control to control the animation speed
timePerFrame = 1/25; % 25 fps
rate = 1/timePerFrame;
rateCtrl = robotics.Rate(rate);  % Adjust rate for desired speed
frames_visu = 'on';

% Loop through each configuration in the joint trajectory
for i = 1:10:numSteps
    % tic
    % Extract the current joint configuration
    currentConfig = jointTrajectory(:, i)';
    
    % Display the robot at the current configuration
    show(robot_visu, currentConfig, 'Frames', frames_visu, 'PreservePlot', false);

    % Wait for the next loop to maintain the rate
    waitfor(rateCtrl);
    % toc
end
