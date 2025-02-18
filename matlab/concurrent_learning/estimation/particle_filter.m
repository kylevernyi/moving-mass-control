clc; clear; close all;

%% System parameters
N = 100;  % Number of particles
T = 50;    % Number of time steps
Q = 1;     % Process noise variance
R = 0.1;   % Measurement noise variance

%% Initialize state and particles
x_true = zeros(1, T);  % True states
y_meas = zeros(1, T);  % Measurements
particles = randn(N, 1);  % Initial particles
weights = ones(N, 1) / N; % Initial weights

%% Simulate system and measurements
x_true(1) = randn;  % Initial state
y_meas(1) = (1/20) * x_true(1)^2 + sqrt(R) * randn;

for k = 2:T
    % System dynamics
    x_true(k) = (1/2) * x_true(k-1) + (25 * x_true(k-1)) / (1 + x_true(k-1)^2) + 8 * cos(1.2 * (k-1)) + sqrt(Q) * randn;
    % Measurement equation
    y_meas(k) = (1/20) * x_true(k)^2 + sqrt(R) * randn;
end

%% Particle filter
x_est = zeros(1, T);  % Estimated states
for k = 2:T
    % Predict step: Sample from state transition
    particles = (1/2) * particles + (25 * particles) ./ (1 + particles.^2) + 8 * cos(1.2 * (k-1)) + sqrt(Q) * randn(N, 1);
    
    % Compute weights based on measurement likelihood
    weights = exp(-0.5 * ((y_meas(k) - (1/20) * particles.^2).^2) / R);
    weights = weights / sum(weights);  % Normalize
    
    % Resample using systematic resampling
    indices = randsample(1:N, N, true, weights);
    particles = particles(indices);
    weights = ones(N, 1) / N;  % Reset weights after resampling
    
    % Estimate state as mean of particles
    x_est(k) = mean(particles);
end

%% Plot results
figure;
plot(1:T, x_true, '--xb', 'LineWidth', 1.5); hold on;
plot(1:T, x_est, '--or', 'LineWidth', 1.5);
legend('True State', 'Estimated State');
xlabel('Time Step'); ylabel('State');
title('Particle Filter State Estimation');
