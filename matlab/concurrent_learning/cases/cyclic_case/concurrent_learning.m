%% This file can turn on or off CL to show the efficacy. 
% it uses the cyclic history stack method of updating the data history 

% MRAC with Concurrent Learning
clear; clc; close all; 
clear all ; clear classes; clear mrac_dynamics;
set(groot, 'DefaultTextInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLegendInterpreter', 'latex'); % plot settings
set(groot, 'DefaultLineLineWidth', 2.1); set(groot, 'DefaultAxesFontSize', 12);
pdf_filename = "CL_results";

%% System Definition
A = [0 1; 4 -2];
B = [0; 1];

% Reference model
Ar = [0 1; -15.21 -5.46];
Br = [0; 1];

% gains
gamma_x = 10;
gamma_R = 1;
K = [0; 0];
Kr = 0;
P = lyap(Ar', eye(2));
Kr_star = 1; % correct known gains to solve for 
K_star = -place(A,B,eig(Ar))'; % solve analytically but flip sign bc place solves A-BK not A+BK (place(A,B,eig(Ar))

% Concurrent learning parameters
p_bar = 8;      % Size of recorded history
epsilon = 0.0001; % how willing we are to accept new CL points
CL_on = 1;
CL_K_gain = .01;
CL_Kr_gain = .01;
global point_added; point_added = [];

% Simulation parameters
dt = 0.001;  % Time step
tf = 50;     % Total simulation time
t = 0:dt:tf;   % Time vector


%% Initial conditions
x = [0;0];
xr = [0;0];

x0 = [x; xr; K; Kr];
dynamicsFunction = @(t,x) mrac_dynamics(t, x, A, B, Ar, Br, P, ... 
    gamma_x, gamma_R, epsilon, p_bar, CL_on, CL_K_gain, CL_Kr_gain,  Kr_star, K_star);
%% Integrate
% x = zeros(length(x0), length(t)); % preallocate state vector
x = x0;
x(:,1) = x0; % set IC
for i = 1:length(t)-1
    % [tt,xx] = ode45(dynamicsFunction, [t(i) t(i+1)], x(:,i), options);
    % [tt, xx] = Runge_Kutta_4(dt, t(i), t(i+1), x(:,i), dynamicsFunction);
    xdot = dynamicsFunction(t(i), x(:,i));

    x(:,i+1) = x(:,i) + xdot*dt;
    % x(:,i+1) = xx(2,:)';
end
x=x';


% Back solve for all intermediary variables useful in analysis
[~,U] = cellfun(dynamicsFunction, num2cell(t), num2cell(x',1),'UniformOutput',0); 
misc_signals = [];
for i = 1:length(U)
    cellArray = U(i);  
    misc_signals(i, :) = cell2mat(cellArray');  % Convert and transpose to row
end

% extract time histories of states
xr = x(:,3:4);
K = x(:,5:6);
Kr = x(:,7);
x = x(:,1:2);

u = misc_signals(:,1);
r = misc_signals(:,2);
min_sing_value_X = misc_signals(:,3);
u_pd = misc_signals(:,4);
u_rm = misc_signals(:,5);
V = misc_signals(:,6);

[~, indices] = ismember(point_added, t); % for seeing when we added points with CL
point_added_indices = indices(indices > 0);



figure; hold on;
plot(t, x(:,1), 'k-', t, xr(:,1),'r--');
title("State 1"); legend(["$x_1$" "$xr_1$"]); grid on;

figure; hold on;
plot(t, x(:,2), 'k-', t, xr(:,2),'r--');
title("State 2"); legend(["$x_2$" "$xr_2$"]); grid on;

figure; hold on;
plot(t, x-xr);
title("Error Signal"); legend(["$e_1$" "$e_2$"]); grid on;

figure; hold on;
k_gains_plot = ones(2,length(t)).*K_star;
plot(t, K, t, Kr, ...
    t, k_gains_plot, '--g', ...
    t, ones(length(t),1 )*Kr_star,'--g');
title("Gains"); legend(["K1" "K2" "Kr" "$K1^*$" "$K2^*$" "$Kr^*$"]); grid on;

figure;
subplot(3,1,1);
plot(t,u, t, u_pd, t, u_rm); ylabel("Control Signal"); legend(["u" "$u_{pd}$" "$u_{rm}$"])
subplot(3,1,2); 
plot(t,r); ylabel("Reference Signal");
subplot(3,1,3); 
plot(t,min_sing_value_X, '-g', t(point_added_indices), min_sing_value_X(point_added_indices), 'ob'); ylabel("Minimum Singular Value");


% lb = 0.5*min([min(eig(P)), min(eig(inv(gamma_x))), inv(gamma_R)]);
% ub = 0.5*max([max(eig(P)), max(eig(inv(gamma_x))), inv(gamma_R)]);
figure; hold on;
plot(t, V, '-g', t(point_added_indices), V(point_added_indices),'ob', 'MarkerFaceColor', 'b')
% plot(t, ones(length(t))*lb, '--r', t, ones(length(t))*ub, '--r');
title("Lyapunov Function");


figure;
plot(t,u, t, u_pd, t, u_rm); ylabel("Control Signal"); legend(["u" "$u_{pd}$" "$u_{rm}$"])

% figure;
% stairs(t(point_added_indices), point_added); 
% title("Points Added To Stack"); xlabel("Time (sec)")


%% PDF export
figHandles = findall(0,'Type','figure'); 
% Save first figure
export_fig(pdf_filename, '-pdf', figHandles(1))
% Loop through figures 2:end
for i = 2:numel(figHandles)
export_fig(pdf_filename, '-pdf', figHandles(i), '-append')
end


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
