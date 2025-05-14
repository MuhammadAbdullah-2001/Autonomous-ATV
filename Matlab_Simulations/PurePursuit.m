clc;
clear;

% Vehicle Parameters
L = 1.2;                    % Wheelbase [m]
v = 2.0;                    % Constant velocity [m/s]
lookahead = 2.5;            % Lookahead distance [m]
dt = 0.05;                  % Time step [s]
total_time = 20;            % Total simulation time [s]

% Initial State [x, y, theta]
state = [0; 0; 0];

% Generate curved path (quarter circle)
theta_path = linspace(0, pi/2, 50);
x_curve = 10 * cos(theta_path - pi/2);
y_curve = 10 * sin(theta_path - pi/2);

% Generate straight path
x_straight = linspace(x_curve(end), x_curve(end)+20, 50);
y_straight = ones(size(x_straight)) * y_curve(end);

% Combine into full path [Nx2]
path = [x_curve', y_curve'; x_straight', y_straight'];



% Store trajectory for plotting
trajectory = zeros(round(total_time/dt), 3);
t = 0;
i = 1;

while t < total_time
    % Pure Pursuit Controller
    delta = pure_pursuit(state, path, lookahead);

    % Kinematic Bicycle Model update
    state = state + dt * [v * cos(state(3));
                          v * sin(state(3));
                          v * tan(delta) / L];

    trajectory(i, :) = state';
    t = t + dt;
    i = i + 1;
end

% Plot Results
figure;
plot(path(:,1), path(:,2), 'k--', 'LineWidth', 1.5); hold on;
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
legend('Reference Path', 'UGV Trajectory');
title('Pure Pursuit Path Tracking');
axis equal; grid on;

% --- Helper Functions ---

function delta = pure_pursuit(x, path, Ld)
    % Convert global path to local vehicle frame
    R = [cos(x(3)), sin(x(3)); -sin(x(3)), cos(x(3))];
    local_path = (path - x(1:2)') * R';

    % Find target point at lookahead distance
    dists = vecnorm(local_path, 2, 2);
    idx = find(dists >= Ld, 1);
    if isempty(idx)
        delta = 0; return;
    end
    goal = local_path(idx, :);

    % Compute curvature and steering angle
    alpha = atan2(goal(2), goal(1));
    delta = atan2(2 * Ld * sin(alpha), Ld^2);
end
