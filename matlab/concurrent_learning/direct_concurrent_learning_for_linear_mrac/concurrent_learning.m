% MRAC with Concurrent Learning
clear; clc; close all; clear all; clear mrac_dynamics;
set(groot, 'DefaultTextInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLegendInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLineLineWidth', 2.1); set(groot, 'DefaultAxesFontSize', 12);  % plot settings
pdf_filename = "example_CL_for_MRAC";

%% Simulation parameters
dt = 0.005;  % Time step
tf = 30;     % Total simulation time
t = 0:dt:tf;   % Time vector

global u_global r_global  V_global nu_global P_kf L  ; % for plotting

%% System Definition
% A = [0 1; 3 -1];
% B = [0; 1]; 
B = 1;
A = 1;

% Reference model
% Ar = [0 1; -4.21 -2.46];
% Br = [0; 1];
Ar = -1;
Br = 0;

%% Uncertainty
Theta = 4;

%% Gains
gamma_x = .05; gamma_R = 1;
K = -place(A,B,eig(Ar))'; Kr = 1;
P = lyap(Ar', eye(1));
Kr_star = 1; % correct known gains to solve for 
K_star = -place(A,B,eig(Ar))'; % correct known gains analytically but flip sign bc place solves A-BK not A+BK (place(A,B,eig(Ar))

%% Concurrent learning parameters
p_bar = 100;      % Max size of recorded history
CL_point_accept_epsilon = 0.1; % how willing we are to accept new CL points
CL_on = 1; % turn on or off concurrent learning
CL_K_gain = .1 / p_bar ; %* inv(gamma_x); % divide by p_bar so we can increase p_bar without making gain super sensitive (normalizes)
CL_Kr_gain = 20 / p_bar; %* inv(gamma_R);
global point_added; point_added = []; % keeps track of when we added a CL point
global min_sv; min_sv = 0;

%% Kalman Filter
Phi_stm = expm([0 1; 0 0]*dt); % state transition matrix
H = [eye(1), zeros(1,1)]; % [1 0] measurement matrix
R_cov = 0.00001; % measurement noise  covariance
Q_cov = diag([1 25]); % process noise covariance (turn this on or off in the dynamics, but good to keep some in the filter)
v_smooth = 5; % number of future measurements to consider while smoothing point j


%% Initial conditions
% x = [1;0];
% xr = [1;0];
x = [1];

x0 = [x; 10]; %x theta_hat_0
dynamicsFunction = @(t,x) mrac_dynamics(t, x, A, B, Ar, Br, P, ... 
    gamma_x, gamma_R, CL_point_accept_epsilon, p_bar, CL_on, CL_K_gain, CL_Kr_gain, Kr_star, K_star, ...
    Q_cov, R_cov, Phi_stm, H, v_smooth, ...
    Theta, K, ...
    true);

%% Integrate
x = x0;
for i = 1:length(t)-1
    xdot = dynamicsFunction(t(i), x(:,i));
    x(:,i+1) = x(:,i) + xdot*dt;
end
x=x';

% extract time histories of states
% xr = x(:,3:4);
% K = x(:,5:6);
% Kr = x(:,7);
theta_hat = x(:,2);
x = x(:,1);


[~, indices] = ismember(point_added, t); % for seeing when we added points with CL
point_added_indices = indices(indices > 0);

figure;
plot(t, x);  title("X over Time");
ylabel("x"); xlabel("Time (sec)"); grid on;

figure; hold on;
plot(t, theta_hat, t, ones(length(t), 1)*Theta, "--" ); title("Uncertainty Estimation");
ylabel("$\theta$"); xlabel("Time (sec)"); grid on; legend(["$\hat\theta$" "$\theta$"]);


% state 1
% figure; hold on;
% plot(t, x(:,1), 'k-', t, xr(:,1),'r--');
% title("State 1"); legend(["$x_1$" "$xr_1$"]); grid on;
% 
% % state 2
% figure; hold on;
% plot(t, x(:,2), 'k-', t, xr(:,2),'r--');
% title("State 2"); legend(["$x_2$" "$xr_2$"]); grid on;
% 
% figure; hold on;
% plot(t, x-xr);
% title("Error Signal"); legend(["$e_1$" "$e_2$"]); grid on;
% 
% % Gains
% figure; hold on;
% k_gains_plot = ones(2,length(t)).*K_star;
% plot(t, K, t, Kr, ...
%     t, k_gains_plot, '--g', ...
%     t, ones(length(t),1 )*Kr_star,'--g', ...
%     t(point_added_indices), K(point_added_indices), 'ob');
% title("Gains"); legend(["K1" "K2" "Kr" "$K1^*$" "$K2^*$" "$Kr^*$"]); grid on;
% 
figure;
subplot(3,1,1);
plot(t(2:end),u_global); ylabel("Control Signal"); legend("u");
subplot(3,1,2); 
plot(t(2:end),r_global); ylabel("Reference Signal");
subplot(3,1,3); 
plot(t,min_sv, '-g', t(point_added_indices), min_sv(point_added_indices), 'ob'); ylabel("Minimum Singular Value");

% 
% % lb = 0.5*min([min(eig(P)), min(eig(inv(gamma_x))), inv(gamma_R)]);
% % ub = 0.5*max([max(eig(P)), max(eig(inv(gamma_x))), inv(gamma_R)]);
figure; hold on;
plot(t(2:end), V_global, '-g', t(point_added_indices), V_global(point_added_indices),'ob', 'MarkerFaceColor', 'b')
% plot(t, ones(length(t))*lb, '--r', t, ones(length(t))*ub, '--r');
title("Lyapunov Function");
disp("# of CL points added: " + num2str(length(point_added)));

figure; hold on;
plot(t(2:end), nu_global, t, x);
legend(["$\hat \nu_1$" "$\hat \nu_2$" "$\nu_1$" ])
title("State Estimation"); xlabel("Time");

% figure; hold on;
% plot(t, sqrt(squeeze(P_kf(1,1,:))), t, sqrt(squeeze(P_kf(2,2,:)))  );
% 
% figure; hold on;
% plot(t(2:end), L(1,:), t(2:end), L(2,:));
% legend(["L1" "L2"]);

%% PDF export
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
