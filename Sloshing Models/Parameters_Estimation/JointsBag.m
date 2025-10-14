%--------------------------------------------------------------------------
% Joint Readings
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------
clear all
close all
clc

bag_folder = fullfile(pwd,"bagfiles");
data = "070725";
file = "2A___NOPT.bag";

bag_path = fullfile(bag_folder, data, file);

bag = rosbag(bag_path);

fs = 500; % Sampling frequency (Hz)
dt = 1/fs; % Time step in seconds


%% 
topic = select(bag, 'Topic', '/comau_smart_six/joint_states');
msgs = readMessages(topic);

% store all forces
q = cellfun(@(m) m.Position, msgs, 'UniformOutput', false);
q1 = cellfun(@(x) x(1), q); % Extract the first value from each cell
q2 = cellfun(@(x) x(2), q); % Extract the second value
q3 = cellfun(@(x) x(3), q);
q4 = cellfun(@(x) x(4), q);
q5 = cellfun(@(x) x(5), q);
q6 = cellfun(@(x) x(6), q);

q_dot =  cellfun(@(m) m.Velocity, msgs, 'UniformOutput', false);
q1_dot = cellfun(@(x) x(1), q_dot); % Extract the first value from each cell
q2_dot = cellfun(@(x) x(2), q_dot); % Extract the second value
q3_dot = cellfun(@(x) x(3), q_dot);
q4_dot = cellfun(@(x) x(4), q_dot);
q5_dot = cellfun(@(x) x(5), q_dot);
q6_dot = cellfun(@(x) x(6), q_dot);

t = (0:length(q1)-1) * dt; % Time vector

figure;
hold on;
plot(t, q1, 'LineWidth', 1.5);
plot(t, q2, 'LineWidth', 1.5);
plot(t, q3, 'LineWidth', 1.5);
plot(t, q4, 'LineWidth', 1.5);
plot(t, q5, 'LineWidth', 1.5);
plot(t, q6, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Position');
title('Joint Positions Over Time');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
grid on;

for i = 1:length(t)
    if i == 1
        qp1(i) = 0;
        qp2(i) = 0;
        qp3(i) = 0;
        qp4(i) = 0;
        qp5(i) = 0;
        qp6(i) = 0;
    else
        dt = t(i)-t(i-1);
        qp1(i) = (q1(i)-q1(i-1))/dt;
        qp2(i) = (q2(i)-q2(i-1))/dt;
        qp3(i) = (q3(i)-q3(i-1))/dt;
        qp4(i) = (q4(i)-q4(i-1))/dt;
        qp5(i) = (q5(i)-q5(i-1))/dt;
        qp6(i) = (q6(i)-q6(i-1))/dt;
    end
end
 
figure;
hold on;
plot(t, qp1, 'LineWidth', 1.5);
plot(t, qp2, 'LineWidth', 1.5);
plot(t, qp3, 'LineWidth', 1.5);
plot(t, qp4, 'LineWidth', 1.5);
plot(t, qp5, 'LineWidth', 1.5);
plot(t, qp6, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Velocities');
title('Joint Velocities Over Time');
legend('$\dot q1$', '$\dot q2$', '$\dot q3$', '$\dot q4$', '$\dot q5$', '$\dot q6$', 'Interpreter', 'Latex');
grid on;




figure;
hold on;
plot(t, q1_dot, 'LineWidth', 1.5);
plot(t, q2_dot, 'LineWidth', 1.5);
plot(t, q3_dot, 'LineWidth', 1.5);
plot(t, q4_dot, 'LineWidth', 1.5);
plot(t, q5_dot, 'LineWidth', 1.5);
plot(t, q6_dot, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Velocities');
title('Joint Velocities Over Time');
legend('$\dot q1$', '$\dot q2$', '$\dot q3$', '$\dot q4$', '$\dot q5$', '$\dot q6$', 'Interpreter', 'Latex');
grid on;

%% TF
% Select the /tf topic containing transformations
tf_topic = select(bag, 'Topic', '/tf');
tf_msgs = readMessages(tf_topic);


num_msgs = length(tf_msgs); % Number of TF messages

% Initialize storage for translation and rotation
translations = cell(num_msgs, 1);
rotations = cell(num_msgs, 1);
timestamps = zeros(num_msgs, 1);

for i = 1:num_msgs
    tf_structs = tf_msgs{i}.Transforms; % Get the list of transforms in the message

    % Assuming you are interested in the first transform in each message
    if ~isempty(tf_structs)
        tf = tf_structs(1); % Take the first transform in the message
        translations{i} = [tf.Transform.Translation.X, ...
                           tf.Transform.Translation.Y, ...
                           tf.Transform.Translation.Z];

        rotations{i} = [tf.Transform.Rotation.X, ...
                        tf.Transform.Rotation.Y, ...
                        tf.Transform.Rotation.Z, ...
                        tf.Transform.Rotation.W];

        timestamps(i) = tf.Header.Stamp.Sec + tf.Header.Stamp.Nsec * 1e-9; % Convert timestamp to seconds
    end
end


valid_idx = ~cellfun(@isempty, translations); % Find non-empty entries
translations = cell2mat(translations(valid_idx)); % Convert to numeric matrix
rotations = cell2mat(rotations(valid_idx)); % Convert to numeric matrix
timestamps = timestamps(valid_idx); % Keep only valid timestamps

% Normalize time to start at 0
timestamps = timestamps - timestamps(1);

figure;
hold on;
plot(timestamps, translations(:,1), 'r', 'LineWidth', 1.5);
plot(timestamps, translations(:,2), 'g', 'LineWidth', 1.5);
plot(timestamps, translations(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Translation (m)');
title('TF Translation Over Time');
legend('X', 'Y', 'Z');
grid on;

figure;
hold on;
plot(timestamps, rotations(:,1), 'r', 'LineWidth', 1.5);
plot(timestamps, rotations(:,2), 'g', 'LineWidth', 1.5);
plot(timestamps, rotations(:,3), 'b', 'LineWidth', 1.5);
plot(timestamps, rotations(:,4), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Quaternion Components');
title('TF Rotation (Quaternion) Over Time');
legend('q_x', 'q_y', 'q_z', 'q_w');
grid on;


