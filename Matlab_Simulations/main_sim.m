clc; clear;

% ------------------------ Waypoint Definition ------------------------
% Create a square (4 sides, 10m each, with a few midpoints = 10 total)

% R = 50;              % Radius in meters
% s = 5;               % Desired spacing in meters
% C = 2 * pi * R;      % Circumference of the circle
% N = floor(C / s);    % Number of points
% delta_theta = 2 * pi / N;   % Angular spacing in radians
% 
% theta = (0:N-1) * delta_theta;  % Angle array
% 
% x = R * cos(theta);   % X coordinates
% y = R * sin(theta);   % Y coordinates
% 
% waypoints = [x', y'];    % Nx2 array of (x, y) points
% Total 10 points
% Parameters
a = 25;                    % Scale factor (width ~50 meters)
t_dense = linspace(0, 2*pi, 1000);  % Densely sample t

% Parametric equations (Lemniscate of Gerono)
x_dense = a * cos(t_dense);
y_dense = a * sin(t_dense) .* cos(t_dense);

% Compute cumulative arc length
dx = diff(x_dense);
dy = diff(y_dense);
ds = sqrt(dx.^2 + dy.^2);
s = [0, cumsum(ds)];  % Arc length at each point

% Desired spacing between points
spacing = 5;                     % meters
total_length = s(end);
num_points = floor(total_length / spacing);

% Generate equally spaced arc lengths
s_uniform = linspace(0, total_length, num_points);

% Interpolate to get equally spaced points
x_uniform = interp1(s, x_dense, s_uniform);
y_uniform = interp1(s, y_dense, s_uniform);

% Store in a Nx2 array
waypoints = [x_uniform', y_uniform'];


% ------------------------ Vehicle Setup ------------------------
state = [20; 0; deg2rad(0)]; % Initial position and heading
v = 2.5;                    % Constant speed [m/s]
dt = 0.1;                   % Time step [s]
lookahead_gain = 1.5;
lookahead_min = 7.0;
lookahead_max = 8.0;
goal_radius = 1.0;          % How close to a waypoint counts as "reached"
v_max = 5.0;                         % [m/s] Max speed of your ATV (realistically measured)
delta_max = deg2rad(45);            % [rad] Max steering angle (typical for small ATVs)

% ------------------------ Visualization Setup ------------------------
figure; hold on; axis equal; grid on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1.5);
scatter(waypoints(:,1), waypoints(:,2), 60, 'r', 'filled');
xlabel('X [m]'); ylabel('Y [m]');
title('Pure Pursuit - Square Waypoint Tracking');

% ------------------------ Simulation Setup ------------------------
trajectory = [];
error_list = [];
max_steps = 1000;
wp_index = 2; % Start with wp1 as current, wp0 as previous

% ------------------------ Main Loop ------------------------
for step = 1:max_steps
    if wp_index > size(waypoints, 1)
        disp('Reached final waypoint. Stopping.');
        break;
    end

    % Waypoints
    prev_wp = waypoints(wp_index - 1, :);
    curr_wp = waypoints(wp_index, :);
    pos = state(1:2);

    % Pure Pursuit
    [target_bearing, status] = calcTargetBearing(curr_wp, prev_wp, pos, ...
        v, lookahead_gain, lookahead_min, lookahead_max);

    % Kinematic bicycle model with steering angle
    L = 0.98;  % Your measured ATV wheelbase in meters

    heading_error = wrapToPi(target_bearing - state(3));

    % Convert heading error to steering angle (Pure Pursuit approximation)
    % δ = arctan(2L sin(α) / lookahead_distance)
    % For this: lookahead_distance = status.lookahead_distance
    

    % Limit steering angle
    
    % Compute desired steering angle from heading error
    delta = atan2(2 * L * sin(heading_error), status.lookahead_distance);

    % Clamp to mechanical steering limit
    delta = max(min(delta, delta_max), -delta_max);
    % Compute desired speed (e.g., based on curvature or constant)
    v_command = v_max;

    % Update state using bicycle model
    theta = state(3);
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = v * tan(delta) / L;

    % Euler integration
    state = state + dt * [x_dot; y_dot; theta_dot];

    % Store
    trajectory = [trajectory; state'];
    error_list = [error_list; status.crosstrack_error];

    % Plot
    plot(pos(1), pos(2), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
    drawnow;

    % Check if reached current waypoint
    if norm(pos - curr_wp') < goal_radius
        wp_index = wp_index + 1;
    end
end

% ------------------------ Final Plot ------------------------
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2);
legend('Path', 'Waypoints', 'Rover Path');

% Optional: Crosstrack error
figure;
plot((1:length(error_list)) * dt, error_list, 'r', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Cross-track Error [m]');
title('Cross-track Error Over Time'); grid on;
