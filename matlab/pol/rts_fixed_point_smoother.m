clear; clc; close all;

%% System Parameters
T = 1; % Sampling time
A = [1 T; 0 1]; % State transition matrix
H = [1 0]; % Measurement matrix
Q = [0.1 0; 0 0.1]; % Process noise covariance
R = 1; % Measurement noise covariance

% Initial state and covariance
x_true = [0; 1]; % True initial state [position; velocity]
x0 = [0; 0]; % Initial estimate
P0 = eye(2); % Initial covariance

% Simulation parameters
N = 50; % Number of time steps
w = mvnrnd([0; 0], Q, N)'; % Process noise
v = sqrt(R) * randn(1, N); % Measurement noise

%% Simulate System
x = zeros(2, N); % True states
y = zeros(1, N); % Observations
x(:,1) = x_true;

for k = 2:N
    x(:,k) = A * x(:,k-1) + w(:,k);
    y(k) = H * x(:,k) + v(k);
end

%% Forward Pass: Kalman Filter
x_f = zeros(2, N); % Filtered states
P_f = zeros(2, 2, N); % Filtered covariances

x_f(:,1) = x0;
P_f(:,:,1) = P0;

for k = 2:N
    % Prediction
    x_pred = A * x_f(:,k-1);
    P_pred = A * P_f(:,:,k-1) * A' + Q;
    
    % Update
    K = P_pred * H' / (H * P_pred * H' + R);
    x_f(:,k) = x_pred + K * (y(k) - H * x_pred);
    P_f(:,:,k) = (eye(2) - K * H) * P_pred;
end

%% Backward Pass: RTS Smoother
x_s = zeros(2, N); % Smoothed states
P_s = zeros(2, 2, N); % Smoothed covariances
x_s(:,N) = x_f(:,N);
P_s(:,:,N) = P_f(:,:,N);

for k = N-1:-1:1
    % Compute RTS smoother gain
    G = P_f(:,:,k) * A' / P_f(:,:,k+1);
    
    % RTS update
    x_s(:,k) = x_f(:,k) + G * (x_s(:,k+1) - A * x_f(:,k));
    P_s(:,:,k) = P_f(:,:,k) + G * (P_s(:,:,k+1) - P_f(:,:,k+1)) * G';
end

%% Plot Results
figure;
subplot(2,1,1);
plot(1:N, x(1,:), 'k', 'LineWidth', 2); hold on;
plot(1:N, y, 'rx', 'MarkerSize', 5);
plot(1:N, x_f(1,:), 'b--', 'LineWidth', 1.5);
plot(1:N, x_s(1,:), 'g', 'LineWidth', 2);
legend('True Position', 'Measurements', 'Filtered', 'Smoothed');
title('Position Estimation');
xlabel('Time Step');
ylabel('Position');

subplot(2,1,2);
plot(1:N, x(2,:), 'k', 'LineWidth', 2); hold on;
plot(1:N, x_f(2,:), 'b--', 'LineWidth', 1.5);
plot(1:N, x_s(2,:), 'g', 'LineWidth', 2);
legend('True Velocity', 'Filtered', 'Smoothed');
title('Velocity Estimation');
xlabel('Time Step');
ylabel('Velocity');
