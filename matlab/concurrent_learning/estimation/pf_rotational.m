clc; clear; close all;
rng("default")

%% System parameters
N = 500;  % Number of particles
T = 200;    % Number of time steps
Q = 0.00001*diag([1 1 1]); % Process noise covariance
R = diag([0.0001, 0.0001, 0.0001]); % Measurement noise covariance
I = diag([1, 1, 1]); % Moment of inertia matrix (assuming a diagonal form for simplicity)

%% Initialize state and particles
omega_true = zeros(3, T);  % True angular velocities
y_meas = zeros(3, T);  % Measurements
% particles = randn(3, N);  % Initial particles
weights = ones(N, 1) / N; % Initial weights
omega_est = zeros(3, T);  % Estimated angular velocities
ESS = zeros(1, T);
% 
%% Initial state and measurement
omega_true(:,1) = 0.05*randn(3,1);  % Initial angular velocity
y_meas(:,1) = omega_true(:,1) + sqrtm(R) * randn(3,1);

particles = omega_true(:,1) + 0.05* randn(3, N); % Small spread around the true state
omega_est(:,1) = mean(particles, 2);


%% Particle filter loop
invI = inv(I);
for k = 2:T
    % Simulate spacecraft dynamics (Euler's equation of motion)
    tau_k = [0.01 * sin(0.1 * k); 0.01 * cos(0.1 * k); 0.01 * sin(0.05 * k)]; % Example external torque
    omega_dot = invI * (-cross(omega_true(:,k-1), I * omega_true(:,k-1)) + tau_k);
    omega_true(:,k) = omega_true(:,k-1) + omega_dot + 0.0001*sqrtm(Q) * randn(3,1);
    
    % Measurement model (gyro readings)
    y_meas(:,k) = omega_true(:,k) + sqrtm(R) * randn(3,1);
    
    % Predict step: Propagate particles using nonlinear dynamics
    for i = 1:N
        omega_dot_p = invI *(-cross(particles(:,i), I * particles(:,i)) +  tau_k);
        particles(:,i) = particles(:,i) + omega_dot_p + sqrtm(Q) * randn(3,1);
    end
    
    % Compute weights based on measurement likelihood
    weights = exp(-0.5 * sum(( (y_meas(:,k) - particles).^2) ./ diag(R)));
    weights = weights / sum(weights);  % Normalize
    
    % Resample using systematic resampling
    indices = randsample(1:N, N, true, weights);
    particles = particles(:, indices);
    weights = ones(N, 1) / N;  % Reset weights after resampling

    ess = 1 / sum(weights.^2);
    ESS(k) = ess;


    % if ess < N / 2
    %     indices = randsample(1:N, N, true, weights);
    %     particles = particles(:, indices);
    % end

    % Estimate state as mean of particles
    omega_est(:,k) = mean(particles, 2);
end

%% Plot results
figure;
for i = 1:3
    subplot(3,1,i);
    plot(1:T, omega_true(i,:), '-b', 'LineWidth', 1.5); hold on;
    plot(1:T, y_meas(i,:), '--r', 'LineWidth', 1.5);
    plot(1:T, omega_est(i,:), '--g', 'LineWidth', 1.5);
    legend(["True" "Meas" "Est"])
    % legend('True Angular Velocity', 'Estimated Angular Velocity');
   ylabel(['ω_' num2str(i) ' (rad/s)']);
end
sgtitle('Particle Filter Estimation of Spacecraft Angular Velocity');
xlabel('Time'); 
% 
% figure;
% plot(1:T, ESS);