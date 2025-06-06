% time,omega_b2i_0,omega_b2i_1,omega_b2i_2,q_b2i_0,q_b2i_1,q_b2i_2,q_b2i_3,q_i2d_0,q_i2d_1,q_i2d_2,q_i2d_3,r_mass_0,r_mass_1,r_mass_2,rdot_mass_0,rdot_mass_1,rdot_mass_2,r_mass_commanded_0,r_mass_commanded_1,r_mass_commanded_2,u_com_0,u_com_1,u_com_2,u_actual_0,u_actual_1,u_actual_2,theta_hat_0,theta_hat_1,theta_hat_2
% clc; close all;

close all;

%% Get csv file
addpath('matlab')
files = dir('logs/*.csv'); % Get all CSV files
if ~isempty(files)
    [~, idx] = max([files.datenum]); % Find the most recent file
    newestFile = strcat('logs/',files(idx).name);
    newestFile = strcat('logs/', "recorded_run.csv");
    data = readtable(newestFile, 'ReadVariableNames', true); % Read data
    fprintf('Loaded file: %s\n', newestFile);
else
    error('No CSV files found in the directory.');
end

q_i2b = [data.q_i2b_0, data.q_i2b_1, data.q_i2b_2, data.q_i2b_3];
C = QuatToDCM(q_i2b);
ypr = squeeze(EulerFromDCM(C))';

%% max torques
m = [260; 260; 268] * 1e-3; % mass matrix
r_max = [63; 63; 97.75] * 1e-3; % maximum offset 
g_B_max = [0; 0; 9.81]; % gravity vectory associated with max torque is aligned with body 
u_max = diag(m) * cross(-g_B_max, r_max); u_max(3) = 97.75*1e-3 * 268*1e-3 * 9.81;
u_max = u_max*1000;
% r_max_check = diag(m)\(cross(g_B_max,u_max)/(norm(g_B_max)^2))

% Form some matrices
data.q_i2d = [data.q_i2d_0 data.q_i2d_1 data.q_i2d_2 data.q_i2d_3];
data.omega_d2i_D = [data.omega_d2i_D_0 data.omega_d2i_D_1 data.omega_d2i_D_2];

data.omega_b2i_B = [data.omega_b2i_0 data.omega_b2i_1 data.omega_b2i_2];
data.omega_b2i_I = zeros(size(data.omega_b2i_B));
data.q_d2b = zeros(length(data.time), 4);
%% Compute omega_b2i_I
for i = 1:length(data.time)
        % data.omega_b2i_I(i,:)=
    omega_b2i_I = quat_mult(quat_mult((q_i2b(i,:)'),[0;data.omega_b2i_B(i,:)']), quat_conj(q_i2b(i,:)'));
    data.omega_b2i_I(i,:)= omega_b2i_I(2:4);


    % data.q_d2b(i,:) = quat_mult(quat_conj(data.q_i2d(i,:)'), quat_conj(data.q_b2i(i,:)')); %//  quat_mult(quat_conj(q_i2d),q_i2b);
end

%% Compute some velocity norms 

omega_p_I = [data.omega_b2i_I(:,1), data.omega_b2i_I(:,2)]; % x and y 
norm_omega_p_I = vecnorm(omega_p_I,2, 2);


%% Plotting
time = data.time / 1000; % to seconds

% Plot \omega_{b2i}
figure; 
subplot(3,1,1); plot(time, data.omega_b2i_0, 'r'); legend('$\omega_{b2i,x}$'); ylabel('$\omega_{b2i}^B$ (rad/s)'); axis tight; hold on;
subplot(3,1,2); plot(time, data.omega_b2i_1, 'g'); legend('$\omega_{b2i,y}$'); ylabel('$\omega_{b2i}^B$ (rad/s)'); axis tight;
subplot(3,1,3); plot(time, data.omega_b2i_2, 'b'); legend('$\omega_{b2i,z}$'); ylabel('$\omega_{b2i}^B$ (rad/s)'); axis tight;
xlabel('Time (s)');  sgtitle('$\omega_{b2i}$');

figure; 
subplot(3,1,1); plot(time, data.omega_d2i_D_0, 'r'); legend('$\omega_{d2i,x}$'); ylabel('$\omega_{d2i}^D$ (rad/s)'); axis tight; hold on;
subplot(3,1,2); plot(time, data.omega_d2i_D_1, 'g'); legend('$\omega_{d2i,y}$'); ylabel('$\omega_{d2i}^D$ (rad/s)'); axis tight;
subplot(3,1,3); plot(time, data.omega_d2i_D_2, 'b'); legend('$\omega_{d2i,z}$'); ylabel('$\omega_{d2i}^D$ (rad/s)'); axis tight;
xlabel('Time (s)');  sgtitle('$\omega_{d2i}^D$');


figure; sgtitle("Euler Angles");
subplot(3,1,1); plot(time, ypr(:,1), 'r'); legend('yaw'); ylabel('yaw (rad)'); hold on; axis tight;
subplot(3,1,2); plot(time, ypr(:,2), 'g'); legend('pitch'); ylabel('pitch (rad)'); axis tight;
subplot(3,1,3); plot(time, ypr(:,3), 'b'); legend('roll'); ylabel('roll (rad)');  axis tight;
xlabel('Time (s)');

% Plot q_{i2b}
figure;
subplot(4,1,1); plot(time, data.q_i2b_0, 'k'); legend('$q_{i2b,0}$'); ylabel('$q_{i2b}$'); axis tight; hold on;
subplot(4,1,2); plot(time, data.q_i2b_1, 'r'); legend('$q_{i2b,1}$'); ylabel('$q_{i2b}$');  axis tight;
subplot(4,1,3); plot(time, data.q_i2b_2, 'g'); legend('$q_{i2b,2}$'); ylabel('$q_{i2b}$');  axis tight;
subplot(4,1,4); plot(time, data.q_i2b_3, 'b'); legend('$q_{i2b,3}$'); ylabel('$q_{i2b}$');  axis tight;
xlabel('Time (s)'); sgtitle('$q_{i2b}$');

% Plot q_{i2d}
figure; 
subplot(4,1,1); plot(time, data.q_i2d_0, 'k');  legend('$q_{i2d,0}$'); ylabel('$q_{i2d}$'); axis tight; hold on;
subplot(4,1,2); plot(time, data.q_i2d_1, 'r'); legend('$q_{i2d,1}$'); ylabel('$q_{i2d}$');axis tight;
subplot(4,1,3); plot(time, data.q_i2d_2, 'g'); legend('$q_{i2d,2}$'); ylabel('$q_{i2d}$');axis tight;
subplot(4,1,4); plot(time, data.q_i2d_3, 'b'); legend('$q_{i2d,3}$'); ylabel('$q_{i2d}$');axis tight;
xlabel('Time (s)');  sgtitle('$q_{i2d}$');


% Position plots
% figure;
% subplot(3,1,1); plot(time, data.r_mass_0*1000, 'r'); legend('$r_{\mathrm{mass},x}$'); ylabel('$r_{\mathrm{mass}}$ (mm)'); hold on;
% subplot(3,1,2); plot(time, data.r_mass_1*1000, 'g'); legend('$r_{\mathrm{mass},y}$'); ylabel('$r_{\mathrm{mass}}$ (mm)');
% subplot(3,1,3); plot(time, data.r_mass_2*1000, 'b'); legend('$r_{\mathrm{mass},z}$'); ylabel('$r_{\mathrm{mass}}$ (mm)');
% xlabel('Time (s)');
% sgtitle('$r_{\mathrm{mass}}$');

% figure; 
% subplot(3,1,1); plot(time, data.rdot_mass_0, 'r'); legend('$\dot{r}_{\mathrm{mass},x}$'); ylabel('$\dot{r}_{\mathrm{mass}}$ (m/s)'); hold on;
% subplot(3,1,2); plot(time, data.rdot_mass_1, 'g'); legend('$\dot{r}_{\mathrm{mass},y}$'); ylabel('$\dot{r}_{\mathrm{mass}}$ (m/s)');
% subplot(3,1,3); plot(time, data.rdot_mass_2, 'b'); legend('$\dot{r}_{\mathrm{mass},z}$'); ylabel('$\dot{r}_{\mathrm{mass}}$ (m/s)');
% xlabel('Time (s)'); 
% sgtitle('$\dot{r}_{\mathrm{mass}}$');

figure;
subplot(3,1,1); plot(time, data.r_mass_commanded_0*1000, '--r', time, data.r_mass_0*1000, 'k'); legend('$r_{\mathrm{mass,cmd},x}$', '$r_{\mathrm{mass},x}$'); ylabel('$r$ (mm)'); axis tight; hold on;
subplot(3,1,2); plot(time, data.r_mass_commanded_1*1000, '--g', time, data.r_mass_1*1000, 'k'); legend('$r_{\mathrm{mass,cmd},y}$', '$r_{\mathrm{mass},y}$'); ylabel('$r$ (mm)'); axis tight;
subplot(3,1,3); plot(time, data.r_mass_commanded_2*1000, '--b', time, data.r_mass_2*1000, 'k'); legend('$r_{\mathrm{mass,cmd},z}$', '$r_{\mathrm{mass},z}$'); ylabel('$r$ (mm)'); axis tight;
xlabel('Time (s)'); sgtitle('$r$ Commanded vs Actual');

% Control inputs
% figure;
% subplot(3,1,1); plot(time, data.u_com_0*1000, 'r'); legend('$u_{\mathrm{com},x}$'); ylabel('$u_{\mathrm{com}}$ (mNm)'); hold on;
% subplot(3,1,2); plot(time, data.u_com_1*1000, 'g'); legend('$u_{\mathrm{com},y}$'); ylabel('$u_{\mathrm{com}}$ (mNm)');
% subplot(3,1,3); plot(time, data.u_com_2*1000, 'b'); legend('$u_{\mathrm{com},z}$'); ylabel('$u_{\mathrm{com}}$ (mNm)');
% xlabel('Time (s)'); sgtitle('$u_{\mathrm{com}}$ (mNm)');

% figure;
% subplot(3,1,1); plot(time, data.u_actual_0*1000, 'r'); legend('$u_{\mathrm{actual},x}$'); ylabel('$u_{\mathrm{actual}}$'); hold on;
% subplot(3,1,2); plot(time, data.u_actual_1*1000, 'g'); legend('$u_{\mathrm{actual},y}$'); ylabel('$u_{\mathrm{actual}}$');
% subplot(3,1,3); plot(time, data.u_actual_2*1000, 'b'); legend('$u_{\mathrm{actual},z}$'); ylabel('$u_{\mathrm{actual}}$');
% xlabel('Time (s)'); sgtitle('$u_{\mathrm{actual}}$ (mNm)');

figure;
subplot(3,1,1); plot(time, data.u_com_0*1000, '--r', time, data.u_actual_0*1000, 'k'); legend('$u_{\mathrm{com},x}$', '$u_{\mathrm{actual},x}$'); ylabel('$u$ (mNm)'); axis tight; hold on;
subplot(3,1,2); plot(time, data.u_com_1*1000, '--g', time, data.u_actual_1*1000, 'k'); legend('$u_{\mathrm{com},y}$', '$u_{\mathrm{actual},y}$'); ylabel('$u$ (mNm)'); axis tight;
subplot(3,1,3); plot(time, data.u_com_2*1000, '--b', time, data.u_actual_2*1000, 'k'); legend('$u_{\mathrm{com},z}$', '$u_{\mathrm{actual},z}$'); ylabel('$u$ (mNm)'); axis tight;
xlabel('Time (s)'); sgtitle('$u$ Commanded vs Actual (mNm)');

% figure;
% subplot(3,1,1); plot(time, data.r_mass_commanded_0*1000, 'r'); legend('$r_{\mathrm{mass,cmd},x}$'); ylabel('$r_{\mathrm{mass,cmd}}$ (mm)'); hold on;
% subplot(3,1,2); plot(time, data.r_mass_commanded_1*1000, 'g'); legend('$r_{\mathrm{mass,cmd},y}$'); ylabel('$r_{\mathrm{mass,cmd}}$ (mm)');
% subplot(3,1,3); plot(time, data.r_mass_commanded_2*1000, 'b'); legend('$r_{\mathrm{mass,cmd},z}$'); ylabel('$r_{\mathrm{mass,cmd}}$ (mm)');
% xlabel('Time (s)'); sgtitle('$r_{\mathrm{mass,cmd}}$');

figure;
subplot(3,1,1); plot(time, data.theta_hat_0*1e3, 'r'); legend('$\hat{\theta}_x$'); ylabel('${\hat{\theta}}$ (mm)'); hold on; axis tight;
subplot(3,1,2); plot(time, data.theta_hat_1*1e3, 'g'); legend('$\hat{\theta}_y$'); ylabel('${\hat{\theta}}$ (mm)'); axis tight;
subplot(3,1,3); plot(time, data.theta_hat_2*1e3, 'b'); legend('$\hat{\theta}_z$'); ylabel('${\hat{\theta}}$ (mm)'); axis tight;
xlabel('Time (s)'); sgtitle('${\hat{\theta}}$ (mm)');


% figure;
% subplot(3,1,1); plot(time, data.u_actual_0*1000, 'r', time, ones(length(time), 1)*u_max(1), 'r--', time, -ones(length(time), 1)*u_max(1), 'r--' ); legend('$u_{\mathrm{actual},x}$'); ylabel('$u_{\mathrm{actual}}$'); hold on;
% subplot(3,1,2); plot(time, data.u_actual_1*1000, 'g', time, ones(length(time), 1)*u_max(2), 'g--', time, -ones(length(time), 1)*u_max(2), 'g--'); legend('$u_{\mathrm{actual},y}$'); ylabel('$u_{\mathrm{actual}}$');
% subplot(3,1,3); plot(time, data.u_actual_2*1000, 'b', time, ones(length(time), 1)*u_max(3), 'b--', time, -ones(length(time), 1)*u_max(3), 'b--'); legend('$u_{\mathrm{actual},z}$'); ylabel('$u_{\mathrm{actual}}$');
% xlabel('Time (s)'); sgtitle('$u_{\mathrm{actual}}$ (mNm) with Max and Min Bounds');

figure; 
subplot(2,1,1); plot(time, norm_omega_p_I, '-k');
title("$\|\omega_p^I\|$ vs $t$"); ylabel('$\|\omega_p^I\|$');  xlabel('Time (s)'); axis tight;

subplot(2,1,2); plot(time, vecnorm(data.omega_b2i_I(:,3),2,2), '-k'); % z body rate represented n the inertial;
title("$\|\omega_z^I\|$ vs $t$");  xlabel('Time (s)'); ylabel("$\|\omega_z^I\|$"); axis tight;



