clear; clc; close all;

%% System Parameters
T = 0.001; % Time step
N = 10000; % Number of time steps

% Moment of Inertia (Assume diagonal for simplicity)
J = diag([2, 3, 4]);
J_inv = inv(J);

% Process and measurement noise
Q = 0.001 * eye(3); % Process noise covariance
R = 0.01 * eye(3);  % Measurement noise covariance

% Initial true state
omega_true = [0.1; 0.2; -0.1];

% Initial estimates
omega0 = [0; 0; 0]; 
P0 = eye(3); 

% Control Torque (constant for simplicity)
u = repmat([0.01; -0.02; 0.01], 1, N);

%% Simulate True Dynamics
omega = zeros(3, N);
y = zeros(3, N);
omega(:,1) = omega_true;

for k = 2:N
    omega_dot = J_inv * (cross(omega(:,k-1), J * omega(:,k-1)) + u(:,k-1));
    omega(:,k) = omega(:,k-1) + T * omega_dot + mvnrnd([0; 0; 0], Q)'; % Process noise
    y(:,k) = omega(:,k) + mvnrnd([0; 0; 0], R)'; % Noisy measurements
end

%% Forward Pass: Extended Kalman Filter (EKF)
omega_f = zeros(3, N);
P_f = zeros(3, 3, N);
omega_f(:,1) = omega0;
P_f(:,:,1) = P0;

for k = 2:N
    % Prediction
    omega_pred = omega_f(:,k-1) + T * J_inv * (cross(omega_f(:,k-1), J * omega_f(:,k-1)) + u(:,k-1));
    F_k = eye(3) + T * J_inv * (skew_symmetric(J * omega_f(:,k-1)) - skew_symmetric(omega_f(:,k-1)) * J); % Jacobian
    P_pred = F_k * P_f(:,:,k-1) * F_k' + Q;

    % Update
    K = P_pred * (P_pred + R)^-1; % Kalman Gain (Assuming full state measurement)
    omega_f(:,k) = omega_pred + K * (y(:,k) - omega_pred);
    P_f(:,:,k) = (eye(3) - K) * P_pred;
end

%% Backward Pass: RTS Smoother
omega_s = zeros(3, N);
P_s = zeros(3, 3, N);
omega_s(:,N) = omega_f(:,N);
P_s(:,:,N) = P_f(:,:,N);

for k = N-1:-1:1
    % Compute RTS smoother gain
    F_k = eye(3) + T * J_inv * (skew_symmetric(J * omega_f(:,k)) - skew_symmetric(omega_f(:,k)) * J);
    G = P_f(:,:,k) * F_k' / P_f(:,:,k+1);

    % RTS update
    omega_s(:,k) = omega_f(:,k) + G * (omega_s(:,k+1) - omega_f(:,k+1));
    P_s(:,:,k) = P_f(:,:,k) + G * (P_s(:,:,k+1) - P_f(:,:,k+1)) * G';
end

omega_dot_true = zeros(3, N-1);

for k = 1:N-1
    omega_dot_est(:,k) = (omega_s(:,k+1) - omega_s(:,k)) / T;
    omega_dot_true(:,k) = J_inv * (cross(omega(:,k), J * omega(:,k)) + u(:,k));
end

%% Plot Estimated vs. True \dot{\omega}
figure;
for i = 1:3
    subplot(3,1,i);
    plot(1:N-1, omega_dot_true(i,:), 'k', 'LineWidth', 1.5); hold on;
    plot(1:N-1, omega_dot_est(i,:), 'g--', 'LineWidth', 1.5);
    legend('True \dot{\omega}', 'Estimated \dot{\omega}');
    title(['\dot{\omega}_', num2str(i), ' Estimate']);
    xlabel('Time Step');
    ylabel('\dot{\omega}');
    grid on;
end


%% Estimate \dot{\omega}
omega_dot_est = zeros(3, N-1);
for k = 1:N-1
    omega_dot_est(:,k) = (omega_s(:,k+1) - omega_s(:,k)) / T;
end

%% Plot Results
figure;
subplot(3,1,1);
plot(1:N, omega(1,:), 'k', 1:N, omega_f(1,:), 'b--', 1:N, omega_s(1,:), 'g', 'LineWidth', 1.5);
legend('True', 'Filtered', 'Smoothed');
title('Angular Velocity - \omega_1');

subplot(3,1,2);
plot(1:N, omega(2,:), 'k', 1:N, omega_f(2,:), 'b--', 1:N, omega_s(2,:), 'g', 'LineWidth', 1.5);
title('Angular Velocity - \omega_2');

subplot(3,1,3);
plot(1:N, omega(3,:), 'k', 1:N, omega_f(3,:), 'b--', 1:N, omega_s(3,:), 'g', 'LineWidth', 1.5);
title('Angular Velocity - \omega_3');

%% Extract Covariance Trace
P_trace_f = zeros(1, N); % Trace of filtered covariance
P_trace_s = zeros(1, N); % Trace of smoothed covariance

for k = 1:N
    P_trace_f(k) = trace(P_f(:,:,k)); % Sum of diagonal elements
    P_trace_s(k) = trace(P_s(:,:,k));
end

%% Plot Covariance Trace
figure;
plot(1:N, P_trace_f, 'b--', 'LineWidth', 1.5); hold on;
plot(1:N, P_trace_s, 'g', 'LineWidth', 2);
legend('Filtered Covariance', 'Smoothed Covariance');
title('Covariance Estimate Over Time');
xlabel('Time Step');
ylabel('Trace of P');
grid on;

figure;
omega_dot_est = [[0;0;0], omega_dot_est];

subplot(3,1, 1);
plot(1:N, omega_dot_est(1,:))
subplot(3,1, 2);
plot(1:N, omega_dot_est(2,:))
subplot(3,1, 3);
plot(1:N, omega_dot_est(3,:))

%% Helper Function: Skew-Symmetric Matrix
function S = skew_symmetric(v)
    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0 ];
end
