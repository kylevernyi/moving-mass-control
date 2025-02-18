clear; clc; close all;

%% Define Constants
T = 0.1; % Time step
N = 100; % Number of time steps
J = diag([5, 4, 3]); % Inertia matrix
J_inv = inv(J);

%% Simulated True Dynamics
omega = zeros(3, N); 
omega_dot = zeros(3, N);
u = 0.1 * randn(3, N); % Random control torque

for k = 1:N-1
    omega_dot(:,k) = J_inv * (cross(omega(:,k), J * omega(:,k)) + u(:,k));
    omega(:,k+1) = omega(:,k) + T * omega_dot(:,k);
end

%% State-Space Model for Estimation
F = [eye(3), T*eye(3); zeros(3,3), eye(3)];
G = [zeros(3,3); J_inv]; % Control input matrix
H = [eye(3), zeros(3,3)]; % Measurement matrix

Q = 1e-4 * eye(6); % Process noise covariance
R = 1e-2 * eye(3); % Measurement noise covariance

%% Generate Noisy Measurements
y = omega + sqrt(R) * randn(3, N);

%% Kalman Filter Initialization
x_f = zeros(6, N); % Filtered state estimate [omega; omega_dot]
P_f = zeros(6,6,N); % Filtered covariance
x_f(:,1) = [y(:,1); zeros(3,1)]; % Initial estimate
P_f(:,:,1) = 0.1 * eye(6);

%% Forward Kalman Filter
for k = 2:N
    % Prediction
    x_pred = F * x_f(:,k-1) + G * u(:,k-1);
    P_pred = F * P_f(:,:,k-1) * F' + Q;

    % Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R);

    % Update
    x_f(:,k) = x_pred + K * (y(:,k) - H * x_pred);
    P_f(:,:,k) = (eye(6) - K * H) * P_pred;
end

%% RTS Smoother Initialization
x_s = x_f; 
P_s = P_f;

%% Backward RTS Smoother
for k = N-1:-1:1
    % Smoothing Gain
    S = P_f(:,:,k) * F' / P_pred;

    % Smoothed State Estimate
    x_s(:,k) = x_f(:,k) + S * (x_s(:,k+1) - x_pred);

    % Smoothed Covariance
    P_s(:,:,k) = P_f(:,:,k) + S * (P_s(:,:,k+1) - P_pred) * S';
end

%% Extract \dot{\omega} Estimates
omega_dot_est = x_s(4:6,:);

%% Plot Results
figure;
for i = 1:3
    subplot(3,1,i);
    plot(1:N, omega_dot(i,:), 'k', 'LineWidth', 1.5); hold on;
    plot(1:N, omega_dot_est(i,:), 'g--', 'LineWidth', 1.5);
    legend('True \dot{\omega}', 'RTS Estimated \dot{\omega}');
    title(['\dot{\omega}_', num2str(i), ' Estimate']);
    xlabel('Time Step');
    ylabel('\dot{\omega}');
    grid on;
end
