%--------------------------------------------------------------------------
% CSV Reader (Comau SmartSix robot)
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it 
%    Date:       February 2025
%--------------------------------------------------------------------------
clear all
close all
clc

%% import 
ws_path = fullfile('..', '..');
kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
% add folder and subfolders
addpath(genpath(kin_path));
% addpath(genpath(odes_path));

addpath(genpath('utils\'));
% addpath(genpath('Data\'));

save_csv   = 0;
import_cartesian = 1;
import_joint = 0;

%% Robot
robot       = create_SmartSix();
% jointOffset = [0, 0, pi/2, 0, 0, 0];
toolOffset  = [0 0 0.08]';

robot_visu = importrobot('comau_smartsix5.urdf.xacro');
robot_visu.DataFormat = 'row'; % Options: 'row', 'column', or 'struct'


% T06_0 = T(Rz(pi), [0.7458; -0.0; 1.3146]);

% qOffset = SmartSix_IK(robot, T06_0, 0);
qOffset = [1.917,-0.326,-2.08,0.0,-1.754,3.39];


% Set the robot configuration
figure()
show(robot_visu, qOffset, 'Frames', 'on', 'PreservePlot', true);
hold on
% Set view and labels for better visualization
title('Comau SmartSix Robot in Initial Joint Configuration');
xlabel('X');
ylabel('Y');
zlabel('Z');
% Set axes limits starting from 0
xlim([-1.5 1.5])
ylim([-1.5 1.5])
zlim([0 2])

%% csv trajectory import 
if import_cartesian
    input_path = "Trajectories/FV_2D_0.3m_20s_360deg.csv";
    
    fileID = fopen(input_path, 'r');
    k = 0;
    total_traj = [];
    
    while ~feof(fileID)
        line = fgetl(fileID); % Read line from file
        if ischar(line)
            k = k + 1;
            data = strsplit(line, ';'); % Split by semicolon
    
            temp_traj.time_from_start = seconds(str2double(data{1}));
            temp_traj.positions = zeros(1, 7);
    
            for i = 2:8 % MATLAB is 1-based index, so adjust accordingly
                temp_traj.positions(i-1) = str2double(data{i});
            end
    
            total_traj = [total_traj; temp_traj];
        end
    end
    
    fprintf('I finished importing the file with %d samples\n', k);
    fclose(fileID);
    
    %% Compute IK
    qOffset = [1.917,-0.326,-2.08,0.0,-1.754,3.393];
    % p_offset = [0;0;0.08];
    p_offset = [0;0;0.106]; 

    T_offset = [eye(3), p_offset; 0 0 0 1];
    
    T06_0 = SmartSix_FK(qOffset');

    T07_0 = T06_0*T_offset;
    
    p06_0 = T06_0(1:3,4);
    R06_0 = T06_0(1:3,1:3);

    p07_0 = T07_0(1:3,4);
    R07_0 = T07_0(1:3,1:3);
    
    % q_sol = zeros(length(total_traj),6);
    q_sol = zeros(6,length(total_traj));
    for i=1:length(total_traj)
        % p07(1) = p07_0(1) + total_traj(i).positions(1);
        % p07(2) = p07_0(2) + total_traj(i).positions(2);
        % p07(3) = p07_0(3) + total_traj(i).positions(3);
        quat = [total_traj(i).positions(7), total_traj(i).positions(4:6)];
        % R07 = R06_0*quat2rotm(total_traj(i).positions(4:7));
        R_traj = quat2rotm(quat);
        p_traj = total_traj(i).positions(1:3);
        T_traj = [R_traj, p_traj'; 0 0 0 1];

        T07 = T07_0*T_traj;

        % R07 = R07_0*quat2rotm(quat);
        % % T07 = [R07, p07'; 0 0 0 1];
        % T07 = [R_traj, p_traj'; 0 0 0 1];
        T_ee = T07*inv(T_offset);
    
        joint_sol = SmartSix_IK(robot,T_ee,0);
        % q_sol(i,:) = joint_sol;
        q_sol(:,i) = joint_sol;
    end
    
    q_sol = unwrap(q_sol');
    q_sol = q_sol';
    

    n = length(total_traj);
    Te = n/500;

end

%% csv joint trajectory import 
if import_joint
    input_path = "Trajectories/path_1A_lenta_joint.csv";
    
    fileID = fopen(input_path, 'r');
    n = 0;
    q_sol = [];
    
    data = readmatrix(input_path, 'Delimiter', ';'); % Read CSV file
    q_sol = data'; % Transpose to get 6 x n matrix
    
    n = size(q_sol, 2);
    
    fprintf('I finished importing the file with %d samples\n', n);
    fclose(fileID);

    Te = n/500;
end

figure
% plot(unwrap(q_sol'))
plot(q_sol')
title("Joints")
xlabel("steps")
ylabel("rad")
legend("q1","q2","q3","q4","q5","q6")
grid on


%% 
animations = 1;

if animations
    numSteps = n; % Number of steps in the trajectory
    jointTrajectory = zeros(6,n);
    for i=1:n
        jointTrajectory(:,i) = [q_sol(1,i), q_sol(2,i), q_sol(3,i), q_sol(4,i), q_sol(5,i), q_sol(6,i)]';
        % jointTrajectory(:,i) = [q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i)]';
    end
    

    % Open a new figure for the animation
    fg = figure;
    fg.WindowState = 'maximized';
    hold on;
    axis equal;
    view(3);
    % view([2 0 0]);
    % view([0 T06_0(1,4) 0]);
    title('Comau Robot Joint Trajectory');
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
    % timePerFrame = Te / n;
    timePerFrame = 0.02;
    rate = 1/timePerFrame;
    rateCtrl = robotics.Rate(rate);  % Adjust rate for desired speed
    
    % Loop through each configuration in the joint trajectory
    for i = 1:10:numSteps
        % tic
        % Extract the current joint configuration
        currentConfig = jointTrajectory(:, i)';
        
        % Display the robot at the current configuration
        show(robot_visu, currentConfig, 'Frames', 'on', 'PreservePlot', false);
        
        
        % Wait for the next loop to maintain the rate
        waitfor(rateCtrl);
        % toc
    end

end
