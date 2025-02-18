% Scalar MRAC with Concurrent Learning
clear; clc; close all; 
clear all ; clear classes; clear knownInvariantJ_concurrentDynamics;
set(groot, 'DefaultTextInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLegendInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLineLineWidth', 2.1); set(groot, 'DefaultAxesFontSize', 12);
pdf_filename = "no_CL_nonlinear";
folder = "eps/";
%% System Definition
A = [zeros(3,3), eye(3); zeros(3,6)];
B = [zeros(3,3); eye(3)];
J = diag([0.0226 0.0226 0.0226]); %Vehicle Inertia Tensor 
Theta = [1; 1.5; .66]*10^-3; %Roff Vector from Center of Rotation to Center of Mass
m = 4.2; % mass in kg

%% Reference model
t_s = 30; % settling time for error dynamics
w_n = 4.6/t_s; % natural frequency 
eta = .9; % damping
l_plus = -eta*w_n + i*w_n*sqrt(1-eta^2);
l_minus = -eta*w_n - i*w_n*sqrt(1-eta^2);

% poles = [-0.1+0.1i, -0.1-0.1i, -0.1*sqrt(2)+0.1i, -0.1*sqrt(2)-0.1i, -0.1+0.15i, -0.1-0.15i ];
position_poles = [l_plus; l_plus; l_plus];
velocity_poles = [l_minus; l_minus; l_minus];
poles = [position_poles ; velocity_poles ]
K = -place(A, B, poles)
Ar = A + B*K;
Kr = pinv(-eye(6) * inv(Ar) * B); % input filter

Br = B*Kr;
P = lyap(Ar', 10*eye(6));

% Adaptation rates
adaptive_on = true;
Gamma = .00001;

% Concurrent learning parameters
CL_on = 1 ; % turn on or off concurrent learning
p_bar = 10;      % Size of recorded history
CL_point_accept_epsilon = 0.08; % how willing we are to accept new CL points
% CL_enable_epsilon = 0.1; % when to disable CL part to prevent drift
CL_gain = 0.1;

global point_added; point_added = [];
global min_sv; min_sv = 0;
% Simulation parameters
dt = 0.001;  % Time step
tf = 30;     % Total simulation time
t = 0:dt:tf;   % Time vector

%% Initial Conditions
sigma = [0.1; -0.1; 0.1]; 
sigma_dot = [0.001; 0.001; 0.001];
x = [sigma; sigma_dot];
% x = zeros(6,1);
xr = zeros(6,1);
theta_hat = zeros(3,1);

x0 = [x; xr; theta_hat];

dynamicsFunction = @(t,x) knownInvariantJ_concurrentDynamics(t, x, A, B, Ar, Br, J, m, Theta, K, Kr, P, Gamma, ...
    adaptive_on, CL_on, p_bar, CL_point_accept_epsilon, CL_gain, dt, true);
backsolvingFunction = @(t,x) knownInvariantJ_concurrentDynamics(t, x, A, B, Ar, Br, J, m, Theta, K, Kr, P, Gamma, ...
    adaptive_on, CL_on, p_bar, CL_point_accept_epsilon, CL_gain, dt, false); % integration phase is false

%% Integrate
% x = zeros(length(x0), length(t)); % preallocate state vector
x = x0;
x(:,1) = x0; % set IC
for i = 1:length(t)-1
    % [tt,xx] = ode45(dynamicsFunction, [t(i) t(i+1)], x(:,i), options);
    [tt, xx] = Runge_Kutta_4(dt, t(i), t(i+1), x(:,i), dynamicsFunction);
    x(:,i+1) = xx(2,:)';

    % xdot = dynamicsFunction(t(i), x(:,i));
    % x(:,i+1) = x(:,i) + xdot*dt;

    % MRP Switching logic
    mrp_vec = x(1:3,i+1);
    mrp_norm = norm(mrp_vec);
    if (mrp_norm) >= 1
        x(1:3,i+1) = -mrp_vec/mrp_norm^2; % Switch MRP 
    end
end
x=x';

% Back solve for all intermediary variables useful in analysis
[~,U] = cellfun(backsolvingFunction, num2cell(t), num2cell(x',1),'UniformOutput',0); 
misc_signals = [];
for i = 1:length(U)
    cellArray = U(i);  
    misc_signals(i, :) = cell2mat(cellArray');  % Convert and transpose to row
end

% extract time histories of states
sigma = x(:,1:3);
sigma_dot = x(:,4:6);
xr = x(:,7:12);
theta_hat = x(:,13:15);
e = [sigma, sigma_dot] -xr;

omega = misc_signals(:,1:3);
u = misc_signals(:,4:6);

[~, indices] = ismember(point_added, t); % for seeing when we added points with CL
point_added_indices = indices(indices > 0);


%% Error signal
figure;
plot(t, e(:,1:3)); 
xlabel("Time (sec)"); ylabel("Error Signal"); title("Error")

%% Kinematics
figure; 
for i = 1:3
    subplot(4,1,i)
    plot(t, sigma(:,i));
    lbl = strcat("$\sigma_",num2str(i), "$");
    ylabel(lbl);
end
subplot(4,1,4)
plot(t, vecnorm(sigma,2,2)); ylabel("$\| \sigma \|$")
xlabel('Time (sec)');
sgtitle("Modified Rodrigues Parameters");

%% Body rates
% Plot angular velocities
figure;
for i = 1:3
    subplot(3,1,i);
    plot(t, (omega(:,i)));
    if i == 1 axis = "x";
    elseif i == 2 axis = "y";
    else axis = "z";
    end
    lbl = strcat("$\omega_",axis, "$");
    ylabel(lbl);
end
xlabel('Time (sec)');
sgtitle("Body Rates (rad/s)");

% Plot torque control signal
figure; hold on;
for i = 1:3
    subplot(3,1,i);
    plot(t, u(:,i));
    if i == 1 axis = "x";
    elseif i == 2 axis = "y";
    else axis = "z";
    end
    lbl = strcat("$L_{R_",axis, "}$");
    ylabel(lbl);
end 
sgtitle("Control signal (Nm)"); xlabel("Time (sec)");

%% Estimated parameters
figure;
plot(t,theta_hat, t, Theta.*ones(3,length(t)) , '--g');
title("Uncertainty Estimations"); ylabel("$\hat\theta$"); xlabel("Time (sec)");
legend(["$\hat\theta_x$" "$\hat\theta_y$" "$\hat\theta_z$" "$\theta_x$" "$\theta_y$" "$\theta_z$"]);

%% SVD
figure; 
min_sv = [0 min_sv];
plot(t,min_sv, '-g', t(point_added_indices), min_sv(point_added_indices), 'ob'); ylabel("Minimum Singular Value");


%% Error 
e_norm = vecnorm(e)
norm_e_norm = norm(e_norm)

% % PDF export
% figHandles = findall(0,'Type','figure'); 
% % Save first figure
% export_fig(pdf_filename, '-pdf', figHandles(1))
% % Loop through figures 2:end
% for i = 2:numel(figHandles)
% export_fig(pdf_filename, '-pdf', figHandles(i), '-append')
% end


%% EPS export
% Loop through each figure and save as .eps
% figHandles = findall(0, 'Type', 'figure');% Get all figure handles
% for i = 1:length(figHandles)
%     figure(figHandles(i));  % Make the figure active
%     titleHandle = get(gca, 'Title');
%     titleText = get(titleHandle, 'String');
% 
%     if length(titleText) == 0
%         titleText = gcf().Children(1).String;
%     end
% 
%     titleText = strrep(titleText, ' ', ''); % get rid of spaces
%     titleText = strrep(titleText, '(', ''); % get rid of parenthesis
%     titleText = strrep(titleText, ')', ''); % get rid of parenthesis
%     titleText = strrep(titleText, '$', ''); % get rid of $
%     titleText = strrep(titleText, '/', ''); % get rid of /
% 
%     if length(titleText) ~= 0
%         filename = strcat(folder, titleText, '.eps');
%         saveas(figHandles(i), filename, 'epsc');  % Save as .eps with color
%     end
% end
