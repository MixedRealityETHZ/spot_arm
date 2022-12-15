clc;
clear;
close all;

% Get data from rosbag
filepath = join([getenv("HOME"), "/data/mixed_reality/", ...
    "2022-12-12-11-28-50.bag"], "");
topic = "/tf";
tf_data = raw_data_from_rosbags(filepath, topic, "tf2_msgs/TFMessage");
tf_data = tf_data{1};
[hand_tfs, hand_Ts, hand_timestamps] = get_transforms(tf_data, 'body', 'hand');
[handreq_tfs, handreq_transforms, handreq_timestamps] = ...
    get_transforms(tf_data, 'base_link', 'hand_request');

% Get vectors
hand_positions = zeros(length(hand_tfs), 3);
hand_quaternions = zeros(length(hand_tfs), 4);
for i = 1:length(hand_tfs)
    hand_positions(i, :) = hand_tfs{i}.position;
    hand_quaternions(i, :) = hand_tfs{i}.quaternion;
end
handreq_positions = zeros(length(handreq_tfs), 3);
handreq_quaternions = zeros(length(handreq_tfs), 4);
for i = 1:length(handreq_tfs)
    handreq_positions(i, :) = handreq_tfs{i}.position;
    handreq_quaternions(i, :) = handreq_tfs{i}.quaternion;
end

% Interpolate
hand_idx = 1;
interp_length = length(handreq_timestamps);
while hand_timestamps(end) <= handreq_timestamps(interp_length)
    interp_length = interp_length - 1;
end
hand_interp_positions = zeros(interp_length, 3);
hand_interp_quaternions = zeros(interp_length, 4);
hand_interp_timestamps = handreq_timestamps(1:interp_length);
for i = 1:interp_length
    while hand_timestamps(hand_idx) <= handreq_timestamps(i)
        hand_idx = hand_idx + 1;
    end
    hand_interp_positions(i, :) = interp_vector(...
        hand_timestamps(hand_idx - 1), ...
        hand_timestamps(hand_idx), ...
        hand_positions(hand_idx - 1, :), ...
        hand_positions(hand_idx, :), handreq_timestamps(i));
    hand_interp_quaternions(i, :) = interp_quaternion(...
        hand_timestamps(hand_idx - 1), ...
        hand_timestamps(hand_idx), ...
        hand_quaternions(hand_idx - 1, :), ...
        hand_quaternions(hand_idx, :), handreq_timestamps(i));
end

% Compute errors
position_errors = handreq_positions(1:interp_length, :) - hand_interp_positions;
rotation_between = quatmultiply(quatconj(hand_interp_quaternions), ...
    handreq_quaternions(1:interp_length, :)); % hand -> handreq
% rotation_between = quatmultiply(quatconj(handreq_quaternions(1:interp_length, :)), ...
%     hand_interp_quaternions); % handreq -> hand
angle_errors = 2.0 * acosd(rotation_between(:, 1));

% Start from first hand timestamp (order here matters)
handreq_timestamps = handreq_timestamps - hand_timestamps(1);
hand_interp_timestamps = hand_interp_timestamps - hand_timestamps(1);
hand_timestamps = hand_timestamps - hand_timestamps(1);
min_timestamp = 50.0;
max_timestamp = 100.0;
hand_indices = hand_timestamps >= min_timestamp & ...
    hand_timestamps <= max_timestamp;
hand_timestamps = hand_timestamps(hand_indices);
hand_positions = hand_positions(hand_indices, :);
hand_quaternions = hand_quaternions(hand_indices, :);
handreq_indices = handreq_timestamps >= min_timestamp & ...
    handreq_timestamps <= max_timestamp;
handreq_timestamps = handreq_timestamps(handreq_indices);
handreq_positions = handreq_positions(handreq_indices, :);
handreq_quaternions = handreq_quaternions(handreq_indices, :);
hand_interp_indices = hand_interp_timestamps >= min_timestamp & ...
    hand_interp_timestamps <= max_timestamp;
hand_interp_timestamps = hand_interp_timestamps(hand_interp_indices);
hand_interp_positions = hand_interp_positions(hand_interp_indices, :);
hand_interp_quaternions = hand_interp_quaternions(hand_interp_indices, :);
position_errors = position_errors(hand_interp_indices, :);
rotation_between = rotation_between(hand_interp_indices, :);
angle_errors = angle_errors(hand_interp_indices);

% Plot position
fig_p = figure('Position', [0, 0, 3600, 1200], 'Name', 'Position');
subplot(3, 1, 1);
xlabel('time');
ylabel('x');
hold on;
plot(hand_timestamps, hand_positions(:, 1));
plot(handreq_timestamps, handreq_positions(:, 1));
subplot(3, 1, 2);
xlabel('time');
ylabel('y');
hold on;
plot(hand_timestamps, hand_positions(:, 2));
plot(handreq_timestamps, handreq_positions(:, 2));
subplot(3, 1, 3);
xlabel('time');
ylabel('z');
hold on;
plot(hand_timestamps, hand_positions(:, 3));
plot(handreq_timestamps, handreq_positions(:, 3));

% Plot orientation
fig_q = figure('Position', [0, 0, 3600, 1200], 'Name', 'Orientation');
subplot(4, 1, 1);
xlabel('time');
ylabel('w');
hold on;
plot(hand_timestamps, hand_quaternions(:, 1));
plot(handreq_timestamps, handreq_quaternions(:, 1));
subplot(4, 1, 2);
xlabel('time');
ylabel('x');
hold on;
plot(hand_timestamps, hand_quaternions(:, 2));
plot(handreq_timestamps, handreq_quaternions(:, 2));
subplot(4, 1, 3);
xlabel('time');
ylabel('y');
hold on;
plot(hand_timestamps, hand_quaternions(:, 3));
plot(handreq_timestamps, handreq_quaternions(:, 3));
subplot(4, 1, 4);
xlabel('time');
ylabel('z');
hold on;
plot(hand_timestamps, hand_quaternions(:, 4));
plot(handreq_timestamps, handreq_quaternions(:, 4));

% Plot errors
fig_error = figure('Position', [0, 0, 3600, 1200], 'Name', 'Errors');
subplot(2, 1, 1);
plot(hand_interp_timestamps, position_errors);
xlabel('timestamp (s)');
ylabel('position error (m)');
legend(["x", "y", "z"], "Location","best");
grid on;
subplot(2, 1, 2);
plot(hand_interp_timestamps, angle_errors);
xlabel('timestamp (s)');
ylabel('angle error (deg)');
grid on;