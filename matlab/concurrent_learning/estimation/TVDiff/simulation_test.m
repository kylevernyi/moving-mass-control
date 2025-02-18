% script to demonstrate small-scale example from paper Rick Chartrand, 
% "Numerical differentiation of noisy, nonsmooth data," ISRN
% Applied Mathematics, Vol. 2011, Article ID 164564, 2011.

load simulation.mat

omega = X(5:7, :);

noise = randn(size(omega)) * sigma_omega;

omega = omega + noise; 

omega_x = omega(1,:);


u = TVRegDiff( omega_x, 100, .02, [], 'small', 1e-6, 0.01, 0, 1 ); hold on;

figure;hold on;
plot(T, [0 omega_dot(1,:)], 'k-');
plot(T, u(2:end), 'or');
legend(["True" "Est"]);
title("Omega_x dot Estimation")
% defaults mean that u = TVRegDiff( noisyabsdata, 500, 0.2 ); would be the
% same
% Best result obtained after 7000 iterations, though difference is minimal
%
% Set last input to 0 to turn off diagnostics

