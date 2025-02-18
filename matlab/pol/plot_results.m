colors = ["k" "r" "b" "g"];
set(groot,'defaultAxesFontSize',20)
set(groot,'defaultAxesFontName','Times New Roman')
set(0,'defaulttextInterpreter','latex')
set(groot,'DefaultFigurePosition',[100 150 800 500])
set(groot,'DefaultAxesPosition','factory')

% Angular Rates of BFF w.r. to DF in BFF vs. Time
figure(1)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,omega_b2d_B(:,k),colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\omega_{{B/D}_"+sprintf("%d}^B",k)+"$",Interpreter='latex')
    sgtitle('$\omega_{B/D}^B$ vs.\ Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$\omega (rad/s)$')
    axis tight;
    %xlim([80 300])
end

% Angular Rates of DF w.r. to Inertial in DF vs. Time
figure(2)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,omega_d2i_D(:,k),colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\omega_{{D/I}_"+sprintf("%d}^D",k)+"$",Interpreter='latex')
    sgtitle('$\omega_{D/I}^D$ vs.\ Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$\omega (rad/s)$')
    axis tight;
    if k == 1
        % ylim([-1 1]*0.1)
    elseif k == 2
        % ylim([-1 1]*0.02)
    else
        % ylim([-0.05 0.2])
    end
end

% Angular Rates of DF w.r. to Inertial in DF vs. Time
figure(3)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,X(k+4,:),colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\omega_{{B/I}_"+sprintf("%d}^B",k)+"$",Interpreter='latex')
    sgtitle('$\omega_{B/I}^B$ vs.\ Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$\omega (rad/s)$')
    axis tight;
    if k == 3
        % ylim([-0.15 0.15])
    else
        % ylim([-0.6 0.6])
    end
end

% Estimation Error vs. Time
figure(4)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,(X(k+7,:)-theta(k))*10^3,colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\tilde{\Theta}_"+sprintf("%d",k)+"$",Interpreter='latex')
    xlabel('$t (s)$')
    ylabel('$\tilde{\Theta} (mm)$')
end
    sgtitle('$\tilde{\Theta}$ vs. Time',Fontsize=30)

% Commanded Mass Position vs. Time
figure(5)
for k = 1:1:3
    subplot(3,1,k)
    plot(T(2:end), r_com(k,:)*10^3, colors(k), Linewidth=1.5, LineStyle="--"); hold on; % commanded 
    plot(T, r_masses_actual(k,:)*10^3, colors(k),Linewidth=1.5); hold on; % actual
    plot(T, zeros(1,length(T)),colors(k),Linewidth=.1, LineStyle="-")
    legend([("m_"+sprintf("%d",k)+"_{com}") , "m_"+sprintf("%d",k)])
    xlabel('$t (s)$')
    ylabel('$Position\ (mm)$',FontSize=15)
    %xlim([0 30])
end
sgtitle('Commanded Mass Positions vs. Time',Fontsize=30);

figure;
for k = 1:1:3
    subplot(3,1,k)
    plot(T(2:end),u_com(k,:)*10^3,colors(k),Linewidth=1.5);hold on;
    legend("\tau_"+sprintf("%d",k))
    xlabel('$t (s)$'); ylabel('$Torque\ (N mm)$',FontSize=15)
end
sgtitle('Control Torque vs. Time',Fontsize=30)

% Quaternion DF w.r. to Inertial vs. Time
% figure(6)
for k = 1:1:4
    subplot(4,1,k)
    plot(T,X(k+10,:),colors(k),Linewidth=1.5);hold on;
    if k == 1
        plot(T,ones(1,length(T)),colors(k)+"--")
    else
        plot(T,zeros(1,length(T)),colors(k)+"--")
    end
    sgtitle('$q_{I\rightarrow D}$ vs. Time',Fontsize=30)
    legend("$q_{{I\rightarrow D}_"+sprintf("%d}",k-1)+"$",Interpreter='latex')
    xlabel('$t (s)$')
    ylabel('$q_{I\rightarrow D}$')
    ylim([-1 1]*1.5)
end

% Quaternion BFF w.r. to Inertial vs. Time
figure(7)
for k = 1:1:4
    subplot(4,1,k)
    plot(T,X(k,:),colors(k),Linewidth=1.5);hold on;
    if k == 1
        plot(T,ones(1,length(T)),colors(k)+"--")
    else
        plot(T,zeros(1,length(T)),colors(k)+"--")
    end
    sgtitle('$q_{I\rightarrow B}$ vs. Time',Fontsize=30)
    legend("$q_{{I\rightarrow B}_"+sprintf("%d}",k-1)+"$",Interpreter='latex')
    xlabel('$t (s)$')
    ylabel('$q_{I\rightarrow B}$')
    ylim([-1 1]*1.5)
end

% Quaternion BFF w.r. to DF vs. Time
figure(8)
for k = 1:1:4
    subplot(4,1,k)
    plot(T,q_d2b(:,k),colors(k),Linewidth=1.5);hold on;
    if k == 1
        plot(T,ones(1,length(T)),colors(k)+"--")
    else
        plot(T,zeros(1,length(T)),colors(k)+"--")
    end
    sgtitle('$q_{D\rightarrow B}$ vs. Time',Fontsize=30)
    legend("$q_{{D\rightarrow B}_"+sprintf("%d}",k-1)+"$",Interpreter='latex')
    xlabel('$t (s)$')
    ylabel('$q_{D\rightarrow B}$')
    ylim([-1 1]*1.5)
    %xlim([50 200])
end

% Angular Rates of BF w.r. to Inertial in I vs. Time
figure(9)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,omega_b2i_I(:,k),colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\omega_{{B/I}_"+sprintf("%d}^I",k)+"$",Interpreter='latex')
    sgtitle('$\omega_{B/I}^I$ vs.\ Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$\omega (rad/s)$')
    axis tight;
    if k == 1
        ylim([-1 1]*0.6)
    elseif k == 2
        ylim([-1 1]*0.5)
    else
        ylim([-0.05 0.2])
    end
end

% figure; hold on;
% for k = 1:3
%     subplot(3,1,k);
%     plot(T(2:end), omega_b2i_B_meas(k,:), colors(k), 'LineWidth',.5, 'LineStyle','-' ); 
%     legend("$\omega_{{B/I}_"+sprintf("%d}^B",k)+"$",Interpreter='latex')
% end
% sgtitle('Measured $\omega_{B/I}^B$ vs.\ Time',Fontsize=30)
% xlabel('$t (s)$')
% ylabel('$\omega (rad/s)$')
% axis tight;

% figure; hold on;
% for k = 1:3
%     subplot(3,1,k)
%     plot(T,r(:,k),colors(k),Linewidth=1.5);hold on;
%     legend("r_"+sprintf("%d",k))
%     xlabel('$t (s)$'); ylabel('$Position\ (mm)$',FontSize=15)
% end
% sgtitle('Error signal vs. Time',Fontsize=30)



figure; hold on;
for k = 1:3
    subplot(3,1,k)
    plot(T, X(k+4,:), 'k-'); hold on; % true
    plot(T(2:end), omega_b2i_B_meas(k,:), '-b'); % meas
    plot(T, omega_hat(k,:), '-r'); % kalman
    plot(T, omega_hat_smooth(k,:), '-g');  % smoothed
    legend(["True" "Meas" "Kalman" "Smoothed"])
end
sgtitle("Omega Estimation"); 

figure; hold on;
for k = 1:3
    subplot(3,1,k)
    plot(T(2:end), omega_dot(k,:), 'k-'); hold on;
    plot(T(1:end), omega_dot_hat(k,:), '--r'); hold on;
    plot(T(1:end), omega_dot_hat_smooth(k,:), '--g'); 
    legend(["True" "Kalman" "Smoothed"]) % "True" 
end
sgtitle("Omega Dot Estimation")


%% SVD
global min_sv point_added;
min_sv = [0 0 min_sv];
[~, indices] = ismember(point_added, T); % for seeing when we added points with CL
point_added_indices = indices(indices > 0);
% figure;  
% plot(T,min_sv, '-g', T(point_added_indices), min_sv(point_added_indices), 'ob'); ylabel("Minimum Singular Value");



% global ESS  omega_b2i_B_est;
% omega_b2i_B_est = omega_b2i_B_est';
% figure; hold on;
% for k = 1:3
%     subplot(3,1,k);
%     hold on;
% 
% 
%     % Plot true, measured, and estimated values
%     plot(T, X(k+4,:), '-k', 'LineWidth', 1.5);  % True state
%     plot(T(2:end), omega_b2i_B_meas(k,:), '--b', 'LineWidth', 1.2);  % Measured
%     plot(T, xhat_original(k,:), '-r', 'LineWidth', 1.2);  % kalman
%     plot(T, xhat(k,:), '-g', 'LineWidth', 1.2);  % smoothed
% 
%     % legend([ "uncertainty (\pm\sigma)" "true" "meas" "est"]);
%     legend(["true" "meas" "kalman" "smoothed"]);
% 
%     xlabel('Time');
%     ylabel(['$\omega_' num2str(k) '$ (rad/s)']);
% end
% sgtitle("True, meas, filtered $\omega$"); xlabel("Time (sec)");

% figure;
% plot(T(2:end), ESS, '-k', 'LineWidth', 1.5);
% xlabel('Time Step'); ylabel('Effective Sample Size');
% title('Effective Sample Size Over Time');
% grid on;




% figure;
% for i = 1:3
%     subplot(3,1,i);
%     histogram(particles(i,:), 30, 'Normalization', 'pdf');
%     hold on;
%     % xline(X(i+4,:), '--r', 'LineWidth', 1.5); % True state
%     xlabel(['$\omega_' num2str(i) '$ (rad/s)']);
%     ylabel('Density');
%     title(['Particle Distribution at Final Time Step ($\omega_' num2str(i) '$)']);
% end


% Shows where CL points were added
figure; hold on;
omega_to_plot = omega_b2d_B(:,1);
plot(T, omega_to_plot,colors(1), T(point_added_indices), omega_to_plot(point_added_indices), 'ob', Linewidth=1.5);
disp("# of CL points added: " + num2str(length(point_added)));

fprintf("Position Error Norm:\n");
fprintf("Meas: %f   Kalman: %f  Smoothed:%f\n\n", norm(e_meas(:,:)),  norm(e_kalman(1:3,:)), norm(e_smoothed(1:3,:)));
fprintf("Velocity Error Norm:\n");
fprintf("Kalman: %f  Smoothed:%f\n\n",  norm(e_kalman(4:6,:)), norm(e_smoothed(4:6,:)));

%% Animation (Uncomment below for 3D animation)
% sim_speed = 3;
%{
pause(5)
figure(10)
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Visualization of Desired Attitude');
view(3); % Set the view to 3D
xlim([-1 1]*1)
ylim([-1 1]*1)
zlim([-1 1]*1)
hold on;
Q_B = X(1:4,:)';
Q_D = X(11:14,:)';

I_Frame = eye(3);

quiver3(0, 0, 0, I_Frame(1,1), I_Frame(2,1), I_Frame(3,1), 'r--', 'LineWidth', 2); % X-axis (red)
quiver3(0, 0, 0, I_Frame(1,2), I_Frame(2,2), I_Frame(3,2), 'g--', 'LineWidth', 2); % Y-axis (green)
quiver3(0, 0, 0, I_Frame(1,3), I_Frame(2,3), I_Frame(3,3), 'b--', 'LineWidth', 2); % Z-axis (blue)

for k = 1:5*sim_speed*2:length(Q_B)

    B_Frame = quat2dcm(Q_B(k,:));

    Bx = quiver3(0, 0, 0, B_Frame(1,1), B_Frame(2,1), B_Frame(3,1), 'r', 'LineWidth', 2); % X-axis (red)
    By = quiver3(0, 0, 0, B_Frame(1,2), B_Frame(2,2), B_Frame(3,2), 'g', 'LineWidth', 2); % Y-axis (green)
    Bz = quiver3(0, 0, 0, B_Frame(1,3), B_Frame(2,3), B_Frame(3,3), 'b', 'LineWidth', 2); % Z-axis (blue)

    D_Frame = quat2dcm(Q_D(k,:));

    Dx = quiver3(0, 0, 0, D_Frame(1,1), D_Frame(2,1), D_Frame(3,1), 'Color',[0.6350 0.0780 0.1840], 'LineWidth', 2); % X-axis (red)
    Dy = quiver3(0, 0, 0, D_Frame(1,2), D_Frame(2,2), D_Frame(3,2), 'Color',[0.4660 0.6740 0.1880], 'LineWidth', 2); % Y-axis (green)
    Dz = quiver3(0, 0, 0, D_Frame(1,3), D_Frame(2,3), D_Frame(3,3), 'Color',[0.3010 0.7450 0.9330], 'LineWidth', 2); % Z-axis (blue)

    % Update the data for the rotated frame plot
    set(Bx, 'UData', B_Frame(1,1), 'VData', B_Frame(2,1), 'WData', B_Frame(3,1));
    set(By, 'UData', B_Frame(1,2), 'VData', B_Frame(2,2), 'WData', B_Frame(3,2));
    set(Bz, 'UData', B_Frame(1,3), 'VData', B_Frame(2,3), 'WData', B_Frame(3,3));

    set(Dx, 'UData', D_Frame(1,1), 'VData', D_Frame(2,1), 'WData', D_Frame(3,1));
    set(Dy, 'UData', D_Frame(1,2), 'VData', D_Frame(2,2), 'WData', D_Frame(3,2));
    set(Dz, 'UData', D_Frame(1,3), 'VData', D_Frame(2,3), 'WData', D_Frame(3,3));
    
    title(sprintf('3D Visualization of Desired Attitude t = %.2f s', ...
        delta_step*k));
    
    pause(delta_step/1000)
    
    if k ~= length(Q_B)
        delete(Bx);delete(By);delete(Bz)
        delete(Dx);delete(Dy);delete(Dz)
    end
end

%}



% function [x_hat_f, P_f] = KalmanFilter(Phi, y, Q, H, R)
%     n = size(Phi, 1); % State dimension.
%     N = length(y); % Number of time steps.
% 
%     % 1. Forward Pass (Kalman Filter)
%     x_hat_f = zeros(n, N); % Forward state estimates.
%     P_f = zeros(n, n, N); % Forward error covariance matrices.
%     K_f = zeros(n, size(y, 2), N-1); % Kalman gain.
% 
%     % Initialize:
%     x_hat_f(:, 1) = zeros(n, 1); % Initial state estimate 
%     P_f(:, :, 1) = eye(n); % Initial error covariance 
% 
%     % Recursion:
%     for k = 1:N-1
%         % Prediction:
%         x_hat_f(:, k+1) = Phi * x_hat_f(:, k) ;
%         P_f(:, :, k+1) = Phi * P_f(:, :, k) * Phi' + Q;
% 
%         % Update:
%         K_f(:, :, k) = P_f(:, :, k+1) * H' / (H * P_f(:, :, k+1) * H' + R);
%         x_hat_f(:, k+1) = x_hat_f(:, k+1) + K_f(:, :, k) * (y(k+1,:)' - H * x_hat_f(:, k+1));
%         P_f(:, :, k+1) = (eye(n) - K_f(:, :, k) * H) * P_f(:, :, k+1);
%     end
% end
% 
% 
% function [x_smooth, P_smooth] = rts_smoother(X, P, Phi, Q)
%     % 2. Backward Pass (RTS Smoother)
%     % X is vector of estimated states over time from forward kalman filter (forward)
%     % P is a tensor of covariance matrices over time  from forward kalman filter (forward)
%     % Phi is state trans matrix
%     % Q is process noise matrix
%     N = length(X); % number of points passed to us
% 
%     x_smooth = X; % Smoothed state estimates.
%     P_smooth = P; % Smoothed error covariance matrices.
%     P_smooth_predict = P; % Predicted Smoothed error covariance matrices.
% 
%     % Initialize:
%     K = zeros(6,6, N);
% 
%     % Recursion:
%     for k = N-1:-1:1
%         % Predicted covariance
%         P_smooth_predict(:,:,k) = Phi * P(:,:,k) * Phi' + Q; 
%         % Compute smoother gain:
%         K(:,:,k) = P_smooth(:, :, k) * Phi' * inv(P_smooth_predict(:, :, k));
% 
%         % Smoothing:   
%         x_smooth(:, k) = x_smooth(:, k) + K(:,:,k) * (x_smooth(:,k+1) - (Phi * x_smooth(:,k))) ; 
%         P_smooth(:, :, k) = P_smooth(:, :, k) + K(:,:,k) * (P_smooth(:,:,k+1) - P_smooth_predict(:,:,k)) * K(:,:,k)';
% 
%     end
% end

