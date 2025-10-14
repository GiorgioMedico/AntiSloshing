numSteps = N+1; % Number of steps in the trajectory
jointTrajectory = zeros(6,N+1);
for i=1:N+1
    jointTrajectory(:,i) = [q_eval(1,i), q_eval(2,i), q_eval(3,i), q_eval(4,i), q_eval(5,i), q_eval(6,i)]';
end

N_eval = 1000;
lut_map = get_p_07_0.map(N_eval); % maps the parametric values to the B-spline values
par_vec = linspace(0, 1, N_eval); % Parametric values ranging from 0 to 1, evenly spaced
val_mat = full(lut_map(par_vec));

fg = figure;
set(fg, 'Name', 'Robot Joint Trajectory Animation', 'NumberTitle', 'off');
fg.WindowState = 'maximized';
plot3(val_mat(1,:),val_mat(2,:),val_mat(3,:),'LineWidth',2)
hold on;
axis equal;
view(3);
title('Robot Joint Trajectory');
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
timePerFrame = t_end / (N+1);
rate = 1/timePerFrame;
rateCtrl = robotics.Rate(rate);  % Adjust rate for desired speed

% Loop through each configuration in the joint trajectory
for i = 1:numSteps
    % tic
    % Extract the current joint configuration
    currentConfig = jointTrajectory(:, i)';
    
    % Display the robot at the current configuration
    show(robot_visu, currentConfig, 'Frames', frames_visu, 'PreservePlot', false);
    
    % Wait for the next loop to maintain the rate
    waitfor(rateCtrl);
    % toc
end