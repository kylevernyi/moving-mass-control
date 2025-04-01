%{ 
WORKING SIMULATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Project:      MSAE Thesis - Adaptive Control & Estimation of Spacecraft 
                            Mass Properties Utilizing Moving Masses
File:         MBS_Simulation_RK4_09_10_2024.m
Author:       Pol Fontdegloria Balaguer
Supervisor:   Dr. Riccardo Bevilacqua
Laboratory:   ADAMUS Laboratory
University:   Embry-Riddle Aeronautical University
Date:         September 10th 2024 
Description:  Simulation of Adaptive Control Law for estimation of Center 
              of Mass of a 5-DOF Testbed under gravity torque due to the
              offset between CoM and CoR. Use of a RK4 integrator to
              propagate the dynamics of the system. This script has a
              predefined desired trajectory as a function of t.
%}

clc;clear;close all; format long; clear global variables;

% Load Directories
% cd('\Users\fontd\OneDrive\Desktop\ERAU (MS)\AE 700\Simulations\5-DOF Simulations\Mass Balancing System');
% addpath('\Users\fontd\OneDrive\Desktop\ERAU (MS)\AE 700\Simulations\5-DOF Simulations\Mass Balancing System\Functions');
% addpath('\Users\fontd\OneDrive\Desktop\ERAU (MS)\AE 700\Simulations\5-DOF Simulations\Mass Balancing System\Functions\Dual Quaternion Functions');
addpath('Functions');
addpath('Functions/DualQuaternionFunctions');

% Call Global Variables
global r_com omega_b2i_I_0 q_i2b_0 q_d2b omega_b2d_B omega_d2i_D omega_b2i_I%omega_wo_noise omega_w_noise
global r_error_signal_rec
% Select Integration Parameters
integ_period = 400;
delta_step = 0.01;
sim_speed = 3;

% Constant definition
% Gains in Control Law u
K = 0.0001; alpha = diag([1,1,1]*3);
% Learning Rate in Estimation Law
gamma = diag([1 1 1])*0.00003;
% Desired Oscillation Amplitude
A = pi/9;
% Moment of Inertia Matrix
Jxx = 0.0226; Jyy = 0.0226; Jzz = 0.0226;
%Jxx = 0.0226; Jyy = 0.0257; Jzz = 0.0266;
J0 = diag([Jxx Jyy Jzz]);
% Simulator mass
M = 4.2;
% Actuator mass
m1 = 0.3; m2 = 0.3; m3 = 0.3;
m = diag([m1,m2,m3]);
% Gravity vector
g_I = [0;0;-9.81];

% Sensor Noise VN-100 (IMU) - Add if needed
SR = 800; % Maximum Sample Rate in Hz
ND = 0.0035*pi/180; % Noise Density of Gyroscope in rad
sigma = ND*sqrt(SR); % Noise Standard Deviation

% Initial offset from CoR to CoM in (m)
theta = [0.9;-1.2;1.7]*10^-3;

% Definition of the Integration Period
t_0 = 0;
dt = delta_step;
t_max = integ_period;
tspan = [t_0:delta_step:t_max]; % in seconds

% Initial Conditions
omega_b2i_B_0 = [0.0888;0.08229;0.13611];
q_i2b_0 = [1;0;0;0];%[0.484125,0.017008,0.125903,-0.86573]';
euler_0 = quat2eul(q_i2b_0','ZYX');
q_i2d_0 = [1;0;0;0];%eul2quat([euler_0(1),0,0],'ZYX')';
theta_hat_0 = [0;0;0];
r_com = zeros(3,1);
% Obtain the initial angular rates of the BFF w.r. to Inertial in the
% Inertial Frame
omega_b2i_I_0 = quat_mult(quat_mult(q_i2b_0,[0;omega_b2i_B_0]),quat_conj(q_i2b_0));
omega_b2i_I_0 = omega_b2i_I_0(2:4);
% Define the initial angular rates of the DF w.r. to Inertial in the DF
% by rotating the vector from the Inertial to DF 
omega_d2i_D_0 = quat_mult(quat_mult(quat_conj(q_i2d_0),[0;0;0;omega_b2i_I_0(3)]),q_i2d_0);


% IC vector State Vector
IC = [q_i2b_0;omega_b2i_B_0;theta_hat_0;q_i2d_0]; 

% Generate Data History Matrix
T = tspan;
X = zeros(length(IC),length(T));
Rs = zeros(length(T),3);
Omega_w_noise = zeros(length(T),3);
Omega_wo_noise = zeros(length(T),3);
tau_ext = zeros(length(T),3);
tau_ext_un = zeros(length(T),3);
tau_ext_norm = zeros(length(T),1);
tau_ext_un_norm = zeros(length(T),1);
Omega_b2d_B = zeros(length(T),3); 

% Input IC in Data History Matrix
X(:,1) = IC;
xin = IC;
Rs(1,:) = r_com';
Omega_w_noise(1,:) = (omega_b2i_B_0 + 0.5*10^-3*randn(3,1))';
Omega_wo_noise(1,:) = omega_b2i_B_0';
Omega_b2d_B(1,:) = [0,0,0]';
Omega_d2i_D(1,:) = omega_d2i_D_0(2:4);
Q_d2b(1,:) = quat_mult(quat_conj(q_i2d_0),q_i2b_0);
Omega_b2i_I(1,:) = omega_b2i_I_0;
h = 0;
% Start of Dynamics Intgegration
for i = 2:1:length(tspan)
    
    time = i*delta_step;

    if time > 50 && h ~= 1 && h ~= 2
        h = 1;
        %xin(11:14) = eul2quat([pi/5 0 0],'YXZ')';
        K = 0.0008; alpha = diag([1,1,1]*3);
        % Learning Rate in Estimation Law
        gamma = diag([0 0 1])*0.0002;
    elseif time > 300 && h ~= 2
        h = 2;
        %xin(11:14) = eul2quat([pi/5 0 0],'YXZ')';
        K = 0.0008; alpha = diag([1,1,1]*3);
        % Learning Rate in Estimation Law
        gamma = 0;
    end

    xkplus1 = rk4_singlestep(@(t,x)mbs_ode_traj(t,x,flag,J0,M,m,g_I,theta,gamma,K,alpha,sigma,A,h),delta_step,time,xin);
    
    Rs(i,:) = r_com';
    %Omega_w_noise(i,:) = omega_w_noise';
    %Omega_wo_noise(i,:) = omega_wo_noise';
    Omega_b2d_B(i,:) = omega_b2d_B;
    Omega_d2i_D(i,:) = omega_d2i_D;
    Omega_b2i_I(i,:) = omega_b2i_I;
    Q_d2b(i,:) = q_d2b;
    X(:,i) = xkplus1;
    xin = xkplus1;

    % Uncomment below to see percentage of completion while sim is running
    %if mod(i,(length(tspan)-1)/1000) == 0
    %clc
    %fprintf(sprintf("%.2f",100*time/t_max))
    %end

end

%% Save Data (for long simulations)
% filename = sprintf('Simulation Data/RK4 Simulation Data/step_%g_time_%g_K_%g_gamma_%g_%g_%g.mat',...
%     delta_step,integ_period,Kd,gamma(1,1),gamma(2,2),gamma(3,3));
% save(filename,"T","X","Rs")

%% Plotting
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
    plot(T,Omega_b2d_B(:,k),colors(k),Linewidth=1.5);hold on;
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
    plot(T,Omega_d2i_D(:,k),colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    legend("$\omega_{{D/I}_"+sprintf("%d}^D",k)+"$",Interpreter='latex')
    sgtitle('$\omega_{D/I}^D$ vs.\ Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$\omega (rad/s)$')
    axis tight;
    if k == 1
        ylim([-1 1]*0.1)
    elseif k == 2
        ylim([-1 1]*0.02)
    else
        ylim([-0.05 0.2])
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
        ylim([-0.15 0.15])
    else
        ylim([-0.6 0.6])
    end
end

% Estimation Error vs. Time
figure(4)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,(X(k+7,:)-theta(k))*10^3,colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--")
    sgtitle('$\tilde{\Theta}$ vs. Time',Fontsize=30)
    legend("$\tilde{\Theta}_"+sprintf("%d",k)+"$",Interpreter='latex')
    xlabel('$t (s)$')
    ylabel('$\tilde{\Theta} (mm)$')
    if k == 3
        ylim([-3 3])
    else
        ylim([-1.5 1.5])
    end
    %xlim([0 30])
   % xlim([4500 5000])
   % ylim([-1 1]*0.005)
end

% Commanded Mass Position vs. Time
figure(5)
for k = 1:1:3
    subplot(3,1,k)
    plot(T,Rs(:,k)*10^3,colors(k),Linewidth=1.5);hold on;
    plot(T,zeros(1,length(T)),colors(k)+"--",Linewidth=1.5)
    legend("m_"+sprintf("%d",k))
    sgtitle('Commanded Mass Positions vs. Time',Fontsize=30)
    xlabel('$t (s)$')
    ylabel('$Position\ (mm)$',FontSize=15)
    ylim([-30 30])
    %xlim([0 30])
end

% Quaternion DF w.r. to Inertial vs. Time
figure(6)
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
    plot(T,Q_d2b(:,k),colors(k),Linewidth=1.5);hold on;
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
    plot(T,Omega_b2i_I(:,k),colors(k),Linewidth=1.5);hold on;
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




%% Animation (Uncomment below for 3D animation)
%%{
% pause(5)
% figure(10)
% axis equal;
% grid on;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('3D Visualization of Desired Attitude');
% view(3); % Set the view to 3D
% xlim([-1 1]*1)
% ylim([-1 1]*1)
% zlim([-1 1]*1)
% hold on;
% Q_B = X(1:4,:)';
% Q_D = X(11:14,:)';
% 
% I_Frame = eye(3);
% 
% quiver3(0, 0, 0, I_Frame(1,1), I_Frame(2,1), I_Frame(3,1), 'r--', 'LineWidth', 2); % X-axis (red)
% quiver3(0, 0, 0, I_Frame(1,2), I_Frame(2,2), I_Frame(3,2), 'g--', 'LineWidth', 2); % Y-axis (green)
% quiver3(0, 0, 0, I_Frame(1,3), I_Frame(2,3), I_Frame(3,3), 'b--', 'LineWidth', 2); % Z-axis (blue)
% 
% for k = 1:5*sim_speed*2:length(Q_B)
% 
%     B_Frame = quat2dcm(Q_B(k,:));
% 
%     Bx = quiver3(0, 0, 0, B_Frame(1,1), B_Frame(2,1), B_Frame(3,1), 'r', 'LineWidth', 2); % X-axis (red)
%     By = quiver3(0, 0, 0, B_Frame(1,2), B_Frame(2,2), B_Frame(3,2), 'g', 'LineWidth', 2); % Y-axis (green)
%     Bz = quiver3(0, 0, 0, B_Frame(1,3), B_Frame(2,3), B_Frame(3,3), 'b', 'LineWidth', 2); % Z-axis (blue)
% 
%     D_Frame = quat2dcm(Q_D(k,:));
% 
%     Dx = quiver3(0, 0, 0, D_Frame(1,1), D_Frame(2,1), D_Frame(3,1), 'Color',[0.6350 0.0780 0.1840], 'LineWidth', 2); % X-axis (red)
%     Dy = quiver3(0, 0, 0, D_Frame(1,2), D_Frame(2,2), D_Frame(3,2), 'Color',[0.4660 0.6740 0.1880], 'LineWidth', 2); % Y-axis (green)
%     Dz = quiver3(0, 0, 0, D_Frame(1,3), D_Frame(2,3), D_Frame(3,3), 'Color',[0.3010 0.7450 0.9330], 'LineWidth', 2); % Z-axis (blue)
% 
%     % Update the data for the rotated frame plot
%     set(Bx, 'UData', B_Frame(1,1), 'VData', B_Frame(2,1), 'WData', B_Frame(3,1));
%     set(By, 'UData', B_Frame(1,2), 'VData', B_Frame(2,2), 'WData', B_Frame(3,2));
%     set(Bz, 'UData', B_Frame(1,3), 'VData', B_Frame(2,3), 'WData', B_Frame(3,3));
% 
%     set(Dx, 'UData', D_Frame(1,1), 'VData', D_Frame(2,1), 'WData', D_Frame(3,1));
%     set(Dy, 'UData', D_Frame(1,2), 'VData', D_Frame(2,2), 'WData', D_Frame(3,2));
%     set(Dz, 'UData', D_Frame(1,3), 'VData', D_Frame(2,3), 'WData', D_Frame(3,3));
% 
%     title(sprintf('3D Visualization of Desired Attitude t = %.2f s', ...
%         delta_step*k));
% 
%     pause(delta_step/1000)
% 
%     if k ~= length(Q_B)
%         delete(Bx);delete(By);delete(Bz)
%         delete(Dx);delete(Dy);delete(Dz)
%     end
% end
% 
% %}

%% Dynamics 

function x_dot = mbs_ode_traj(t,u,flag,J0,M,m,g_I,theta,gamma,K,alpha,sigma,A,h)

% Reference Global Variables
global r_com omega_b2i_I_0 omega_b2d_B omega_d2i_D q_d2b omega_b2i_I %omega_wo_noise omega_w_noise
global r_error_signal_rec
% Retrieve State Variables
q_i2b = u(1:4); q_i2b = q_i2b/norm(q_i2b);
omega_b2i_B = u(5:7);
theta_hat = u(8:10);
q_i2d = u(11:14); %q_i2d = q_i2d/norm(q_i2d)
%omega_wo_noise = u(4:6);
%omega_b2i_B = u(4:6); % + sigma*randn(3,1); 
%omega_w_noise = omega_b2i_B;
%q = [q0;q1;q2;q3]; q = q/norm(q);
%theta_hat = [theta_hat1;theta_hat2;theta_hat3];
%q_d = [q0_d;q1_d;q2_d;q3_d];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predefined Omega and Quaternion Trajectory w.r. to Inertial Frame
%q_d2i = eul2quat([A*sin(t), A*sin(t), ...
%                  euler_0(3) + omega_b2i_I_0(3)*t],"XYZ")';%*exp(-0.01*t);
b = 0.01*pi;
f = 0.01;
%omega_b2i_I = quat2rotm(q_i2b')*omega_b2i_B;
omega_b2i_I = quat_mult(quat_mult(q_i2b,[0;omega_b2i_B]),quat_conj(q_i2b));
omega_b2i_I = omega_b2i_I(2:4);
% Angular rates of DF w.r. Inertial in Inertial Frame
omega_d2i_I = [0;0;omega_b2i_I_0(3)];%*exp(-0.005*t);
% Transformation Rotation of Angular Rates of the DF w.r. to Inertial from
% Inertial to DF

omega_d2i_D = quat_mult(quat_mult(quat_conj(q_i2d),[0;omega_d2i_I]),q_i2d);
omega_d2i_D = omega_d2i_D(2:4);%+ [0;0*A*sin(b*t)*exp(-f*t);0];
    T_trans = 30;
    rot_trans = pi/9;
if h == 1 && t < T_trans + 50
    omega_d2i_D = omega_d2i_D + [0;rot_trans*pi/(2*T_trans)*sin(pi/T_trans*(t-50));0];%0.05*exp(-0.1*(t-50))
elseif t == 138.5
    %q_i2d;
    %d = quat2eul(q_i2d','ZYX');
    %d(2)*180/pi;
elseif h == 2 && t < T_trans + 300
    omega_d2i_D = omega_d2i_D + [0;-rot_trans*pi/(2*T_trans)*sin(pi/T_trans*(t-300));0];
end
%
%omega_dot_d2i_I = [-A*b*sin(b*t);0;0];
omega_dot_d2i_D = [0;0*A*exp(-f*t)*(f*sin(b*t) - b*cos(b*t));0];
omega_dot_d2i_I = quat_mult(quat_mult(q_i2d,[0;omega_dot_d2i_D]),quat_conj(q_i2d));
omega_dot_d2i_I = omega_dot_d2i_I(2:4);
%omega_dot_d2i_D = quat_mult(quat_mult(q_d2i,[0;omega_dot_d2i_I]),quat_conj(q_d2i));
omega_dot_d2i_B = quat_mult(quat_mult(quat_conj(q_i2b),[0;omega_dot_d2i_I]),q_i2b);
omega_dot_d2i_B = omega_dot_d2i_B(2:4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quaternion Error from Body to Desired
q_d2b = quat_mult(quat_conj(q_i2d),q_i2b);
% Omega Desired w.r. Inertial in Body Fixed Frame (BFF)
omega_d2i_B = quat_mult(quat_mult(quat_conj(q_d2b),[0;omega_d2i_D]),q_d2b);
omega_d2i_B = omega_d2i_B(2:4);
% Omega Body w.r. Desired in BFF
omega_b2d_B = omega_b2i_B - omega_d2i_B;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of r
r = omega_b2d_B + alpha*q_d2b(2:4);
r_error_signal_rec = [r_error_signal_rec r];
% Update of J(t) as a function of new mass position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = J0; %+ M*diag(r_com.*r_com);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation of Gravity Vector from Inertial to BFF
g_B = quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
g_B = g_B(2:4);
% Skew Matrix of Gravity Vector in BFF
g_b_x = skew(g_B);
% Phi definition as in ref[DOI: 10.2514/1.60380]
Phi = -M*g_b_x;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Torque as Designed by Lyapunov Analysis
P = (eye(3)-(g_B*g_B')/(norm(g_B)^2));
u_com = P*cross(omega_b2i_B,J*omega_b2i_B) - P*Phi*theta_hat...
    + J*P*(omega_dot_d2i_B + cross(omega_d2i_B,omega_b2d_B)...
    - 0.5*alpha*(skew(q_d2b(2:4)) + q_d2b(1)*eye(3))*omega_b2d_B)...
    - K*P*r - diag((P*theta_hat).^2)*r;


% f1 = P*diag((theta_hat).^2)
% f2 = diag(P*(theta_hat).^2)
% theta-theta_hat
% (theta-theta_hat).^2
% 
% P*[0;0;1]
% g_B
% f3 = diag((P*theta_hat).^2)
% f4 = diag(theta_hat)*P*theta_hat
% det(f1)
% f2 = P*[theta_hat(1)^2;0;0]
% f3= P*[0;theta_hat(2)^2;0]
% f4 = P*[0;0;theta_hat(3)^2]
%P*diag(theta_hat.^2)
%det(P*diag(theta_hat.^2))

%u_com = P*u_com;
u_com_check = quat_mult(quat_mult(q_i2b,[0;u_com]),quat_conj(q_i2b))
%u_com_check(4) = 0;
%u_com = quat_mult(quat_mult(quat_conj(q_i2b),u_com_check),q_i2b);
%u_com = u_com(2:4);
% Transformation of u_com to Commanded Positions as in ref[DOI: 10.2514/1.60380]
r_com = m\(cross(g_B,u_com)/(norm(g_B)^2));
% Real control applied to the system w/ moving masses
u = m*(cross(-g_B,r_com))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define first order ODE's 
% q_dot of BFF w.r. to Inertial Frame
q_i2b_dot = 0.5*quat_mult(q_i2b,[0;omega_b2i_B]);
% Omega_dot Body w.r. Inertial in Body Frame
omega_b2i_B_dot = J\(- cross(omega_b2i_B,J*omega_b2i_B) + Phi*theta + u_com);
% Theta_hat_dot
theta_hat_dot = gamma*(Phi'*r);%gamma*(Phi'*omega_b2d_B);%
% q_dot of DF w.r. to Inertial Frame

q_i2d_dot = 0.5*quat_mult(q_i2d,[0;omega_d2i_D]);

x_dot = [q_i2b_dot;omega_b2i_B_dot;theta_hat_dot;q_i2d_dot];

end
