clear all; close all; clc;
rng(1)
N = 100; % Points
dt = 0.01;

A = [0 1; 0 0]; % State matrix.
Phi = expm(A*dt);
H = [1 0]; % Observation matrix (example - adjust as needed).
Q = 0.1 * eye(2); % Process noise covariance.
R = 0.1; % Measurement noise covariance.
t = linspace(0, 10*pi,N);
y =  0.1*(t) + R*randn(1, N); % Example measurements (100 time steps, adjust as needed).
T = 1:N;
[xhat , P] = KalmanFilter(Phi, y, Q, H, R);

[x_smooth, P_smooth] = rts_smoother(xhat, P, Phi, R);

plot(t, 0.1*(t), t, y, t, xhat(1,:),  t, x_smooth(1,:));
legend('True', 'Measurements', 'Kalman', 'Smoothed');
xlabel('Time'); ylabel('State'); title("Position Estimate")


figure;
plot(t, xhat(2,:), t, x_smooth(2,:), t, 0.1*ones(1,length(t)));
legend('Kalman', 'Smoothed', 'True');
xlabel('Time'); ylabel('Velocity'); title("Velocity Estimate")

figure;
P1 = reshape(P(1,1,:), 1, N);
P2 = reshape(P(2,2,:), 1, N);
P1_smooth = reshape(P_smooth(1,1,:), 1, N);
P2_smooth  = reshape(P_smooth(2,2,:), 1, N);
plot(t, P1, t, P2, t,P1_smooth, t,P2_smooth);
legend('P1', 'P2', 'P1_s', 'P2_s'); title("Variance")


function [x_hat_f, P_f] = KalmanFilter(Phi, y, Q, H, R)
    n = size(Phi, 1); % State dimension.
    N = length(y); % Number of time steps.
    
    % 1. Forward Pass (Kalman Filter)
    x_hat_f = zeros(n, N); % Forward state estimates.
    P_f = zeros(n, n, N); % Forward error covariance matrices.
    K_f = zeros(n, N-1); % Kalman gain.
    
    % Initialize:
    x_hat_f(:, 1) = [sin(0); .1*cos(0)]; % Initial state estimate 
    P_f(:, :, 1) = 10*eye(n); % Initial error covariance 
    
    % Recursion:
    for k = 1:N-1
        % Prediction:
        x_hat_f(:, k+1) = Phi * x_hat_f(:, k) ;
        P_f(:, :, k+1) = Phi * P_f(:, :, k) * Phi' + Q;
    
        % Update:
        K_f(:, k) = P_f(:, :, k+1) * H' / (H * P_f(:, :, k+1) * H' + R);
        x_hat_f(:, k+1) = x_hat_f(:, k+1) + K_f(:, k) * (y(k+1) - H * x_hat_f(:, k+1));
        P_f(:, :, k+1) = (eye(n) - K_f(:, k) * H) * P_f(:, :, k+1);
    end
end

function [x_smooth, P_smooth] = rts_smoother(X, P, Phi, Q)
% 2. Backward Pass (RTS Smoother)
% X is vector of estimated states over time from forward kalman filter (forward)
% P is a tensor of covariance matrices over time  from forward kalman filter (forward)
% Phi is state trans matrix
% Q is process noise matrix
N = length(X); % number of points passed to us

x_smooth = X; % Smoothed state estimates.
P_smooth = P; % Smoothed error covariance matrices.
P_smooth_predict = P; % Predicted Smoothed error covariance matrices.

% Initialize:
K = zeros(2,2, N);

% Recursion:
for k = N-1:-1:1
    % Predicted covariance
    P_smooth_predict(:,:,k) = Phi * P(:,:,k) * Phi' + Q; 
    % Compute smoother gain:
    K(:,:,k) = P_smooth(:, :, k) * Phi' / P_smooth_predict(:, :, k);

    % Smoothing:   
    x_smooth(:, k) = x_smooth(:, k) + K(:,:,k) * (x_smooth(:,k+1) - (Phi * x_smooth(:,k))) ; 
    P_smooth(:, :, k) =P_smooth(:, :, k) + K(:,:,k) * (P_smooth(:,:,k+1) - P_smooth_predict(:,:,k)) * K(:,:,k)';

end

% x_smooth = flip(x_smooth);
% P_smooth = flip(P_smooth, 3);
end
