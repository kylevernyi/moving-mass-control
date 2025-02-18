function [states_dot] = SpacecraftDynamics(t,states,tmax,time_vec,data_t,mu_sun,mu_moon,initial_date,omega_Earth_vec,m_B,M_B,Cd,M_D,mu,J2,r_Earth,K_S,M_TM,Kp,Kd,Kp_TM,Kd_TM,GRACE_C_Accelerations,GRACE_D_Accelerations,GRACE_C_Pose,GRACE_D_Pose,GRACE_LRI,Q_prime,R)

%--------------------------------------------------------------------------
%
%   RELATIVE SPACECRAFT STATES 
%
%    states(1:4,:): omega_hat_B_BD real part   (Spacecraft Angular Velocity)
%    states(5:8,:): omega_hat_B_BD dual part   (Spacecraft Angular Linear Velocity)
%   states(9:12,:): q_hat_BD real part         (Spacecraft Angular Position)
%  states(13:16,:): q_hat_BD dual part         (Spacecraft Angular Linear Position)
%
%  states(17:20,:): omega_hat_I_DI real part   (Spacecraft Angular Velocity)
%  states(21:24,:): omega_hat_I_DI dual part   (Spacecraft Angular Linear Velocity)
%  states(25:28,:): q_hat_DI real part         (Spacecraft Angular Position)
%  states(29:32,:): q_hat_DI dual part         (Spacecraft Angular Linear Position)
%
%--------------------------------------------------------------------------

global count omega_hat_dot_B_BD_data f_hat_B_g_data f_hat_B_gradg_data 
global f_hat_B_j2_data dualforce_B_data r_I_DI_vec_data r_I_BI_vec_data
global T_DI_check_data T_BD_check_data attitude_Q_norm_data invex
global rotationcheck_data r_B_BD_vec_data r_dot_B_BD_vec_data
global r_ddot_B_BD_vec_data Euler_data a_hat_B_g_data a_hat_B_j2_data
global a_hat_B_gradg_data omega_hat_dot_TM_TMB_data f_hat_TM_g_data
global a_hat_TM_g_data f_hat_TM_j2_data a_hat_TM_j2_data 
global f_hat_TM_gradg_data a_hat_TM_gradg_data dualforce_TM_data
global r_TM_TMB_vec_data r_dot_TM_TMB_vec_data r_ddot_TM_TMB_vec_data
global f_hat_TM_coupled_data a_hat_TM_coupled_data f_hat_TM_u_data
global inindex f_hat_B_External_data GRACEFOCheckRelativePos_data
global GRACEFOCheckRelativePos_data2 q_hat_DI_data q_hat_BD_data
global q_hat_DI_mag_data q_hat_BD_mag_data a_B_j2_vec_data a_I_j2_vec_data
global v_kepler_data r_kepler_data q_DI_error_data q_DI_kepler_data
global a_I_B_mag_error_data invexindex t_all f_hat_B_u_data
global Theta_TMB_vec_data RelativeRange_Error_data
global DesiredFramePosition_Error_data BodyFixedFramePosition_Error_data
global DQ_DI_attitude_data DQ_BI_attitude_data TestMassFramePosition_data
global R_ITM_data q_DI_data q_BI_data Voltage_data f_hat_TM_u_command_data
global VoltageNoise_data q_hat_TMB_data r_B_TMB_noise_data r_B_TMB_vec_data
global omega_hat_tilde_TM_TMB_data q_hat_tilde_TMB_plus_data difference_sum

%% States |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

%----------------------------------------------------
% Turn on and off linear and/or angluar states


% Body Fixed Frame States
omega_hat_B_BD = [states(1:4,:),states(5:8,:)];
      q_hat_BD = [states(9:12,:),states(13:16,:)];
      % r_B_BD_unnorm = 2.*Qmult(Qconj(q_hat_BD(:,1)),q_hat_BD(:,2));
      q_hat_BD(:,1) = q_hat_BD(:,1)./Qnorm(q_hat_BD(:,1));
      % q_hat_BD(:,2) =  (1/2).*Qmult(q_hat_BD(:,1),r_B_BD_unnorm);

      q_hat_BD(:,2) = (eye(4,4) - ((q_hat_BD(:,1).*(q_hat_BD(:,1)'))./(Qnorm(q_hat_BD(:,1)).^2)))*q_hat_BD(:,2);

      

% Desired Frame States
omega_hat_D_DI = [states(17:20,:),states(21:24,:)];
      q_hat_DI = [states(25:28,:),states(29:32,:)];
      % r_D_DI_unnorm = 2.*Qmult(Qconj(q_hat_DI(:,1)),q_hat_DI(:,2));
      q_hat_DI(:,1) = q_hat_DI(:,1)./Qnorm(q_hat_DI(:,1));
      % q_hat_DI(:,2) =  (1/2).*Qmult(q_hat_DI(:,1),r_D_DI_unnorm);

      q_hat_DI(:,2) = (eye(4,4) - ((q_hat_DI(:,1).*(q_hat_DI(:,1)'))./(Qnorm(q_hat_DI(:,1)).^2)))*q_hat_DI(:,2);


%----------------------------------------------------
%% Desired Frame Dynamics

% Solving for Vectors

r_D_DI = 2.*Qmult(Qconj(q_hat_DI(:,1)),q_hat_DI(:,2));

r_I_DI = 2.*Qmult(q_hat_DI(:,2),Qconj(q_hat_DI(:,1)));
r_I_DI_vec = r_I_DI(2:4,:);

% Forces of Desired Frame

if t == data_t(:,inindex + 1)
    inindex = inindex + 1;

else
    inindex = inindex;
end


a_hat_D_external = [[0;GRACE_C_Accelerations(4:6,inindex)],[0;GRACE_C_Accelerations(1:3,inindex)]];
% a_hat_D_external = zeros(4,2);
f_hat_D_External = DQmatrixmult(M_D,a_hat_D_external);

% Dual force from gravity
[f_hat_D_g,a_hat_D_g] = DQgravitationalforce(M_D,mu,r_D_DI);
[f_hat_D_gradg,a_hat_D_gradg] = DQgravitygradienttorque(M_D,mu,r_D_DI);
[f_hat_D_j2,a_hat_D_j2] = DQj2perturbation(M_D,mu,J2,r_Earth,r_I_DI,q_hat_DI);


% % Dual force summation
% f_hat_D = f_hat_D_g + f_hat_D_gradg + f_hat_D_j2;
% f_hat_D = f_hat_D_g + f_hat_D_j2;
f_hat_D = f_hat_D_g + f_hat_D_gradg + f_hat_D_j2 + f_hat_D_External;
% f_hat_D = f_hat_D_g;


% Dynamics

omega_hat_dot_D_DI = DF_DYN_DQ(M_D,f_hat_D,omega_hat_D_DI);

% omega_hat_dot_D_DI(:,1) = zeros(4,1);

q_hat_dot_DI = (1/2).*DQmult(q_hat_DI,omega_hat_D_DI);

states_dot(17:20,:) = omega_hat_dot_D_DI(:,1);
states_dot(21:24,:) = omega_hat_dot_D_DI(:,2);
states_dot(25:28,:) = q_hat_dot_DI(:,1);
states_dot(29:32,:) = q_hat_dot_DI(:,2);

%----------------------------------------------------


%% Spacecraft Dynamics ||||||||||||||||||||||||||||||||||||||||||||||||||||

% GETTING VECTORS FROM QUATERNIONS

r_B_BD = 2.*Qmult(Qconj(q_hat_BD(:,1)),q_hat_BD(:,2));
r_B_BD_vec = r_B_BD(2:4,:);

q_hat_BI = DQmult(q_hat_DI,q_hat_BD);

r_I_BI = 2.*Qmult(q_hat_BI(:,2),Qconj(q_hat_BI(:,1)));
r_I_BI_vec = r_I_BI(2:4,:);

r_B_BI = 2.*Qmult(Qconj(q_hat_BI(:,1)),q_hat_BI(:,2));

%--------------------------------------------------------------------------
% BODY-FIXED FRAME

% Dual Velocity

omega_hat_B_DI = DQmult(DQconj(q_hat_BD),DQmult(omega_hat_D_DI,q_hat_BD));

omega_hat_B_BI = omega_hat_B_BD + omega_hat_B_DI;


%--------------------------------------------------------------------------
% Approximation of nongravitational accelerations

% [a_GRACE_approximated] = GRACENongravitationalAccelerations(t,m_B,Cd,mu_sun,mu_moon,initial_date,r_I_BI_vec,omega_Earth_vec,omega_hat_B_BI,q_hat_BI);
% 
% a_hat_B_External = [[0;a_GRACE_approximated],zeros(4,1)];
% 
% f_hat_B_External = DQmatrixmult(M_B,a_hat_B_External);

%--------------------------------------------------------------------------
% Nongravitational data from GRACE-FO

a_hat_B_external = [[0;GRACE_D_Accelerations(4:6,inindex)],[0;GRACE_D_Accelerations(1:3,inindex)]];
% a_hat_B_external = zeros(4,2);
f_hat_B_External = DQmatrixmult(M_B,a_hat_B_external);

%--------------------------------------------------------------------------


% Dual force from gravity
[f_hat_B_g,a_hat_B_g] = DQgravitationalforce(M_B,mu,r_B_BI);
[f_hat_B_gradg,a_hat_B_gradg] = DQgravitygradienttorque(M_B,mu,r_B_BI);
[f_hat_B_j2,a_hat_B_j2] = DQj2perturbation(M_B,mu,J2,r_Earth,r_I_BI,q_hat_BI);


% f_hat_B_gradg = zeros(4,2);
% f_hat_B_j2 = zeros(4,2);
% f_hat_B_External = zeros(4,2);


% Controller
f_hat_B_u = PDcontroller_GuiVukovich(Kp,Kd,M_B,f_hat_B_g,f_hat_B_gradg,f_hat_B_j2,omega_hat_B_BD,omega_hat_B_DI,omega_hat_dot_D_DI,q_hat_BD,r_B_BD_vec);


% % Dual force summation
dualforce_B = f_hat_B_g + f_hat_B_gradg + f_hat_B_j2 + f_hat_B_External;
% dualforce_B = f_hat_B_g + f_hat_B_gradg + f_hat_B_j2 + f_hat_B_u;
% dualforce_B = f_hat_B_g + f_hat_B_gradg + f_hat_B_j2;
% dualforce_B = f_hat_B_g + f_hat_B_j2 + f_hat_B_u;
% dualforce_B = zeros(4,2);


% Spacecraft Relative Dyanmics

q_hat_dot_BD = (1/2).*DQmult(q_hat_BD,omega_hat_B_BD);

[omega_hat_dot_B_BD] = SC_REL_DYN_DQ(M_B,dualforce_B,omega_hat_B_BD,omega_hat_B_DI,omega_hat_dot_D_DI,q_hat_BD);


% omega_hat_dot_B_BD(:,1) = zeros(4,1);

states_dot(1:4,:) = omega_hat_dot_B_BD(:,1);
states_dot(5:8,:) = omega_hat_dot_B_BD(:,2);
states_dot(9:12,:) = q_hat_dot_BD(:,1);
states_dot(13:16,:) = q_hat_dot_BD(:,2);


% Clean up for plots

r_dot_B_BD = omega_hat_B_BD(:,2);
r_dot_B_BD_vec = r_dot_B_BD(2:4,:);

r_ddot_B_BD = omega_hat_dot_B_BD(:,2);
r_ddot_B_BD_vec = r_ddot_B_BD(2:4,:);




%% Test Mass Relative Dynamics ||||||||||||||||||||||||||||||||||||||||||||

omega_hat_TM_TMB = [states(33:36,:),states(37:40,:)];
q_hat_TMB = [states(41:44,:),states(45:48,:)];
% r_TM_TMB_unnorm = 2.*Qmult(Qconj(q_hat_TMB(:,1)),q_hat_TMB(:,2));
q_hat_TMB(:,1) = q_hat_TMB(:,1)./Qnorm(q_hat_TMB(:,1)); % Normalize attitude quaternion
% q_hat_TMB(:,2) =  (1/2).*Qmult(q_hat_TMB(:,1),r_TM_TMB_unnorm);

q_hat_TMB(:,2) = (eye(4,4) - ((q_hat_TMB(:,1).*(q_hat_TMB(:,1)'))./(Qnorm(q_hat_TMB(:,1)).^2)))*q_hat_TMB(:,2);

%----------------------------------------------------


omega_hat_TM_BI = DQmult(DQconj(q_hat_TMB),DQmult(omega_hat_B_BI,q_hat_TMB));


omega_hat_dot_B_DI = DQmult(DQconj(q_hat_BD),DQmult(omega_hat_dot_D_DI,q_hat_BD));

% omega_hat_dot_B_BI = omega_hat_dot_B_BD + omega_hat_dot_B_DI;

omega_hat_dot_B_BI = DF_DYN_DQ(M_B,dualforce_B,omega_hat_B_BI);

%----------------------------------------------------

r_B_TMB = 2.*Qmult(q_hat_TMB(:,2),Qconj(q_hat_TMB(:,1)));
r_B_TMB_vec = r_B_TMB(2:4,:);

r_TM_TMB = 2.*Qmult(Qconj(q_hat_TMB(:,1)),q_hat_TMB(:,2));
r_TM_TMB_vec = r_TM_TMB(2:4,:);


q_hat_TMI = DQmult(q_hat_BI,q_hat_TMB);

r_I_TMI = 2.*Qmult(q_hat_TMI(:,2),Qconj(q_hat_TMI(:,1)));
r_I_TMI_vec = r_I_TMI(2:4,:);

r_TM_TMI = 2.*Qmult(Qconj(q_hat_TMI(:,1)),q_hat_TMI(:,2));
r_TM_TMI_vec = r_TM_TMI(2:4,:);

% Finding coupled dual force on test mass
Theta_TMB_vec = AttitudeQuaternions2LinearizedEuler(Qconj(q_hat_TMB(:,1)));
coupledforcemoment = K_S*[r_B_TMB_vec;Theta_TMB_vec]; 
% Theta_TMB_vec = AttitudeQuaternions2LinearizedEuler(q_hat_TMB(:,1));
% coupledforcemoment = K_S*[r_TM_TMI_vec;Theta_TMB_vec]; 
F_S_vec = coupledforcemoment(1:3,:);
M_S_vec = coupledforcemoment(4:6,:);
f_hat_TM_coupled = [[0;F_S_vec],[0;M_S_vec]];
a_hat_TM_coupled = DQmatrixmult(inv(M_TM),f_hat_TM_coupled);

% Dual forces on test mass
[f_hat_TM_g,a_hat_TM_g] = DQgravitationalforce(M_TM,mu,r_TM_TMI);
[f_hat_TM_gradg,a_hat_TM_gradg] = DQgravitygradienttorque(M_TM,mu,r_TM_TMI);
[f_hat_TM_j2,a_hat_TM_j2] = DQj2perturbation(M_TM,mu,J2,r_Earth,r_I_TMI,q_hat_TMI);


% f_hat_TM_gradg = zeros(4,2);
% f_hat_TM_j2 = zeros(4,2);
% f_hat_TM_coupled = zeros(4,2);


% Add sensing noise
SensingNoise = 1;

if SensingNoise == 1
    r_B_TMB_noise = r_B_TMB_vec + (((2.5*(10^-10))*sqrt(10/2)).*randn(3,1));
    r_TM_TMB_vec_noise = r_TM_TMB_vec + (((2.5*(10^-10))*sqrt(10/2)).*randn(3,1));
    q_hat_TMB_noise = [q_hat_TMB(:,1),(1/2).*Qmult([0;r_B_TMB_noise],q_hat_TMB(:,1))];
elseif SensingNoise == 0
    r_B_TMB_noise = r_B_TMB_vec;
    q_hat_TMB_noise = q_hat_TMB;
    r_TM_TMB_vec_noise = r_TM_TMB_vec;
end

% Kalmn filtered states

q_hat_tilde_TMB = reshape(states(49:56,:),[4,2]);
b_hat_tilde_omega = reshape(states(57:64,:),[4,2]);
P = reshape(states(65:208,:),[12,12]);

[q_hat_dot_tilde_TMB,b_hat_dot_tilde_omega,P_dot,
    q_hat_tilde_TMB_plus,b_hat_tilde_omega_plus,P_plus,
    omega_hat_tilde_TM_TMB] = ...
    DQFilipeKalmanFilter(q_hat_TMB_noise, ...
    q_hat_tilde_TMB,b_hat_tilde_omega,P,Q_prime,R);

states(57:64,:) = reshape(b_hat_tilde_omega_plus,[8,1]);

%
states_dot(49:56,:) = reshape(q_hat_dot_tilde_TMB,[8,1]);
states_dot(57:64,:) = reshape(b_hat_dot_tilde_omega,[8,1]);
% if t > 288
%     P_dot = zeros(12,12);
% else
%     P_dot = P_dot;
% end
states_dot(65:208,:) = reshape(P_dot,[144,1]);

% Control from Measured States
MeasuredStates = 1;
% 0: Actual States
% 1: Filtered/Estimated

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_hat_tilde_TMB_plus(1,1) = SolveForAttitudeQuaternionScaler(q_hat_tilde_TMB_plus(:,1));

q_hat_tilde_TMB_plus(1,2) = -(q_hat_tilde_TMB_plus(2:4,1)')*q_hat_tilde_TMB_plus(2:4,2)./q_hat_tilde_TMB_plus(1,1);

% r_check = 2.*Qmult(Qconj(q_hat_tilde_TMB_plus(:,1)),q_hat_tilde_TMB_plus(:,2));

% q_hat_tilde_TMB_plus(1,1) = q_hat_TMB(1,1);
% q_hat_tilde_TMB_plus(1,2) = q_hat_TMB(1,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if MeasuredStates == 1;
    % Dual force from PD-type controller
    f_hat_TM_u_TM_command = PDcontroller_GuiVukovich_TM(Kp_TM,Kd_TM,M_TM,f_hat_TM_g,f_hat_TM_gradg,f_hat_TM_j2,f_hat_TM_coupled,omega_hat_tilde_TM_TMB,omega_hat_TM_BI,omega_hat_dot_B_BI,q_hat_tilde_TMB_plus,r_TM_TMB_vec_noise);
    
    % Converting test mass control commands to the body frame
    f_hat_B_u_TM_command = DQswap(DQmult(q_hat_tilde_TMB_plus,DQmult(DQswap(f_hat_TM_u_TM_command),DQconj(q_hat_tilde_TMB_plus)))); % I believe this is correct
    % f_hat_B_u_TM_command = DQswap(DQmult(DQconj(q_hat_TMB),DQmult(DQswap(f_hat_TM_u_TM_command),q_hat_TMB))); 



    % Perfect Commands
    f_hat_TM_u_TM_command_ideal = PDcontroller_GuiVukovich_TM(Kp_TM,Kd_TM,M_TM,f_hat_TM_g,f_hat_TM_gradg,f_hat_TM_j2,f_hat_TM_coupled,omega_hat_TM_TMB,omega_hat_TM_BI,omega_hat_dot_B_BI,q_hat_TMB_noise,r_TM_TMB_vec_noise);

    f_hat_B_u_TM_command_ideal = DQswap(DQmult(q_hat_TMB_noise,DQmult(DQswap(f_hat_TM_u_TM_command_ideal),DQconj(q_hat_TMB_noise))));

%     difference_command = sum(abs(f_hat_B_u_TM_command_ideal - f_hat_B_u_TM_command));

    difference_command = [Qnorm(q_hat_TMB(:,1)),Qnorm(q_hat_tilde_TMB_plus(:,1))];

    

elseif MeasuredStates == 0;
    % Dual force from PD-type controller
    f_hat_TM_u_TM_command = PDcontroller_GuiVukovich_TM(Kp_TM,Kd_TM,M_TM,f_hat_TM_g,f_hat_TM_gradg,f_hat_TM_j2,f_hat_TM_coupled,omega_hat_TM_TMB,omega_hat_TM_BI,omega_hat_dot_B_BI,q_hat_TMB_noise,r_TM_TMB_vec_noise);
    
    % Converting test mass control commands to the body frame
    f_hat_B_u_TM_command = DQswap(DQmult(q_hat_TMB_noise,DQmult(DQswap(f_hat_TM_u_TM_command),DQconj(q_hat_TMB_noise)))); % I believe this is correct
    % f_hat_B_u_TM_command = DQswap(DQmult(DQconj(q_hat_TMB),DQmult(DQswap(f_hat_TM_u_TM_command),q_hat_TMB))); 

    difference_command = [0,0];
end

Noise = 1; 
% 0: No Electrode Noise
% 1: Electrode Noise

SteppedCommand = 0;
% 0: Continuous Command
% 1: Stepped Command

[F_vec,T_vec,Voltage,VoltageNoise] = ElectrodeForceTorque(f_hat_B_u_TM_command(2:4,1),f_hat_B_u_TM_command(2:4,2),Noise,SteppedCommand);

f_hat_TM_u = [[0;F_vec],[0;T_vec]];

dualforce_TM = f_hat_TM_g + f_hat_TM_gradg + f_hat_TM_j2 + f_hat_TM_coupled + f_hat_TM_u;
% dualforce_TM = f_hat_TM_g + f_hat_TM_gradg + f_hat_TM_j2 + f_hat_TM_u;
% dualforce_TM = f_hat_TM_g + f_hat_TM_gradg + f_hat_TM_j2 + f_hat_TM_coupled;
% dualforce_TM = f_hat_TM_g + f_hat_TM_gradg + f_hat_TM_j2;
% dualforce_TM = f_hat_TM_g + f_hat_TM_gradg + f_hat_TM_j2 + f_hat_TM_coupled + f_hat_TM_u_command;



% Test Mass Relative Dynamics

q_hat_dot_TMB = (1/2).*DQmult(q_hat_TMB,omega_hat_TM_TMB);

[omega_hat_dot_TM_TMB] = SC_REL_DYN_DQ(M_TM,dualforce_TM,omega_hat_TM_TMB,omega_hat_TM_BI,omega_hat_dot_B_BI,q_hat_TMB);

states_dot(33:36,:) = omega_hat_dot_TM_TMB(:,1);
states_dot(37:40,:) = omega_hat_dot_TM_TMB(:,2);
states_dot(41:44,:) = q_hat_dot_TMB(:,1);
states_dot(45:48,:) = q_hat_dot_TMB(:,2);

% Clean up for plots

r_dot_TM_TMB = omega_hat_TM_TMB(:,2);
r_dot_TM_TMB_vec = r_dot_TM_TMB(2:4,:);

r_ddot_TM_TMB = omega_hat_dot_TM_TMB(:,2);
r_ddot_TM_TMB_vec = r_ddot_TM_TMB(2:4,:);


% R_ID = quat2rotm(Qconj(q_hat_DI(:,1))');
% R_IB = quat2rotm(Qconj(q_hat_BI(:,1))');
% 
% R_ID_GRACE = quat2rotm(Qconj(GRACE_C_Pose(1:4,inindex))');
% R_IB_GRACE = quat2rotm(Qconj(GRACE_D_Pose(1:4,inindex))');

% R_ID = quat2rotm(q_hat_DI(:,1)');
% R_IB = quat2rotm(q_hat_BI(:,1)');
% R_ITM = (1/2)*trace(quat2rotm(q_hat_TMI(:,1)') - eye(3));
% 
% R_ID_GRACE = quat2rotm(GRACE_C_Pose(1:4,inindex)');
% R_IB_GRACE = quat2rotm(GRACE_D_Pose(1:4,inindex)');
% 
% 
% DQ_DI_attitude = abs(((1/2)*trace(R_ID_GRACE - eye(3))) - ((1/2)*trace(R_ID - eye(3))));
% DQ_BI_attitude = abs(((1/2)*trace(R_IB_GRACE - eye(3))) - ((1/2)*trace(R_IB - eye(3))));

R_ITM = zeros(3,3);
DQ_DI_attitude = zeros(3,1);
DQ_BI_attitude = zeros(3,1);

DesiredFramePosition_Error = norm(GRACE_C_Pose(5:7,inindex) - r_I_DI_vec);
BodyFixedFramePosition_Error = norm(GRACE_D_Pose(5:7,inindex) - r_I_BI_vec);
TestMassFramePosition = r_I_TMI_vec;


q_DI = q_hat_DI(:,1);
q_BI = q_hat_BI(:,1);


%% Clean Up |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

% count = count + 1;
% if count == 1 || rem(count - 1 ,4) == 0

% [~,timesize] = size(time_vec);
% if t == time_vec(:,invex) && invex <= timesize
if t == time_vec(:,invex)
    invex = invex + 1;

    % Spacecraft
    omega_hat_dot_B_BD_data = [omega_hat_dot_B_BD_data;[omega_hat_dot_B_BD(:,1)',omega_hat_dot_B_BD(:,2)']];
    f_hat_B_g_data = [f_hat_B_g_data;[f_hat_B_g(:,1)',f_hat_B_g(:,2)']];
    a_hat_B_g_data = [a_hat_B_g_data;a_hat_B_g(:,1)'];
    f_hat_B_j2_data = [f_hat_B_j2_data;[f_hat_B_j2(:,1)',f_hat_B_j2(:,2)']];
    a_hat_B_j2_data = [a_hat_B_j2_data;a_hat_B_j2(:,1)'];
    f_hat_B_gradg_data = [f_hat_B_gradg_data;[f_hat_B_gradg(:,1)',f_hat_B_gradg(:,2)']];
    a_hat_B_gradg_data = [a_hat_B_gradg_data;a_hat_B_gradg(:,2)'];
    f_hat_B_External_data = [f_hat_B_External_data;[f_hat_B_External(:,1)',f_hat_B_External(:,2)']];
    f_hat_B_u_data = [f_hat_B_u_data;[f_hat_B_u(:,1)',f_hat_B_u(:,2)']];
    dualforce_B_data = [dualforce_B_data;[dualforce_B(:,1)',dualforce_B(:,2)']];
    r_I_DI_vec_data = [r_I_DI_vec_data;r_I_DI_vec'];
    r_I_BI_vec_data = [r_I_BI_vec_data;r_I_BI_vec'];
    r_B_BD_vec_data = [r_B_BD_vec_data;r_B_BD_vec'];
    r_dot_B_BD_vec_data = [r_dot_B_BD_vec_data;r_dot_B_BD_vec'];
    r_ddot_B_BD_vec_data = [r_ddot_B_BD_vec_data;r_ddot_B_BD_vec'];

    q_DI_data = [q_DI_data,q_DI];
    q_BI_data = [q_BI_data,q_BI];
    
    % Test Mass Values
    omega_hat_dot_TM_TMB_data = [omega_hat_dot_TM_TMB_data;[omega_hat_dot_TM_TMB(:,1)',omega_hat_dot_TM_TMB(:,2)']];
    f_hat_TM_g_data = [f_hat_TM_g_data;[f_hat_TM_g(:,1)',f_hat_TM_g(:,2)']];
    a_hat_TM_g_data = [a_hat_TM_g_data;a_hat_TM_g(:,1)'];
    f_hat_TM_j2_data = [f_hat_TM_j2_data;[f_hat_TM_j2(:,1)',f_hat_TM_j2(:,2)']];
    a_hat_TM_j2_data = [a_hat_TM_j2_data;a_hat_TM_j2(:,1)'];
    f_hat_TM_gradg_data = [f_hat_TM_gradg_data;[f_hat_TM_gradg(:,1)',f_hat_TM_gradg(:,2)']];
    a_hat_TM_gradg_data = [a_hat_TM_gradg_data;a_hat_TM_gradg(:,2)'];
    f_hat_TM_coupled_data = [f_hat_TM_coupled_data;[f_hat_TM_coupled(:,1)',f_hat_TM_coupled(:,2)']];
    a_hat_TM_coupled_data = [a_hat_TM_coupled_data;[a_hat_TM_coupled(:,1)',a_hat_TM_coupled(:,2)']];
    f_hat_TM_u_data = [f_hat_TM_u_data;[f_hat_TM_u(:,1)',f_hat_TM_u(:,2)']];
    f_hat_TM_u_command_data = [f_hat_TM_u_command_data;[f_hat_TM_u_TM_command(:,1)',f_hat_TM_u_TM_command(:,2)']];
    dualforce_TM_data = [dualforce_TM_data;[dualforce_TM(:,1)',dualforce_TM(:,2)']];
    r_TM_TMB_vec_data = [r_TM_TMB_vec_data;r_TM_TMB_vec'];
    r_dot_TM_TMB_vec_data = [r_dot_TM_TMB_vec_data;r_dot_TM_TMB_vec'];
    r_ddot_TM_TMB_vec_data = [r_ddot_TM_TMB_vec_data;r_ddot_TM_TMB_vec'];
    Theta_TMB_vec_data = [Theta_TMB_vec_data,Theta_TMB_vec];
    q_hat_TMB_data = [q_hat_TMB_data;[q_hat_TMB(:,1)',q_hat_TMB(:,2)']];
    r_B_TMB_vec_data = [r_B_TMB_vec_data;r_B_TMB_vec'];
    r_B_TMB_noise_data = [r_B_TMB_noise_data;r_B_TMB_noise'];

    Voltage_data = [Voltage_data,Voltage];
    VoltageNoise_data = [VoltageNoise_data,VoltageNoise];

    RelativeRange_Error = GRACE_LRI(1,invex) - norm(r_B_BD_vec);

    DQ_DI_attitude_data = [DQ_DI_attitude_data,DQ_DI_attitude];
    DQ_BI_attitude_data = [DQ_BI_attitude_data,DQ_BI_attitude];
    DesiredFramePosition_Error_data = [DesiredFramePosition_Error_data,DesiredFramePosition_Error];
    BodyFixedFramePosition_Error_data = [BodyFixedFramePosition_Error_data,BodyFixedFramePosition_Error];
    RelativeRange_Error_data = [RelativeRange_Error_data,RelativeRange_Error];
    TestMassFramePosition_data = [TestMassFramePosition_data,TestMassFramePosition];
    R_ITM_data = [R_ITM_data,R_ITM];

    % Kalman filter
    omega_hat_tilde_TM_TMB_data = [omega_hat_tilde_TM_TMB_data;[omega_hat_tilde_TM_TMB(:,1)',omega_hat_tilde_TM_TMB(:,2)']];
    q_hat_tilde_TMB_plus_data = [q_hat_tilde_TMB_plus_data;[q_hat_tilde_TMB_plus(:,1)',q_hat_tilde_TMB_plus(:,2)']];
    difference_sum = [difference_sum;difference_command];


end

t_all = [t_all,t];



clc
fprintf('Simulation Time = %.2f sec\n',t);
fprintf('Completion Percentage = %.2f %%\n',(t/tmax)*100);


end