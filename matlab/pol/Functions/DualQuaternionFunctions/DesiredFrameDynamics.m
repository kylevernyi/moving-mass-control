function [states_dot] = DesiredFrameDynamics(t,states,tmax,time_vec,M_D,mu,J2,r_Earth)

%--------------------------------------------------------------------------
%
%   DESIRED FRAME STATES 
%
%    states(1:4,:): omega_hat_D_DI real part   (Desired Frame Angular Velocity)
%    states(5:8,:): omega_hat_D_DI dual part   (Desired Frame Angular Linear Velocity)
%   states(9:12,:): q_hat_DI real part         (Desired Frame Angular Position)
%  states(13:16,:): q_hat_DI dual part         (Desired Frame Angular Linear Position)
%
%--------------------------------------------------------------------------

global invex r_I_DI_vec_data

%--------------------------------------------------------------------------

omega_hat_D_DI = [states(1:4,:),states(5:8,:)];
q_hat_DI = [states(9:12,:),states(13:16,:)];

%% Solving for Vectors

r_D_DI = 2.*Qmult(Qinv(q_hat_DI(:,1)),q_hat_DI(:,2));

r_I_DI = Qvectorrotation(r_D_DI,q_hat_DI(:,1));
r_I_DI_vec = r_I_DI(2:4,:);

q_hat_BD = [[1;zeros(3,1)],zeros(4,1)];

%% Forces of Desired Frame

% Dual force from gravity
[f_hat_D_g,a_hat_D_g] = DQgravitationalforce(M_D,mu,r_D_DI);
[f_hat_D_gradg,a_hat_D_gradg] = DQgravitygradienttorque(M_D,mu,r_D_DI);
[f_hat_D_j2,a_hat_D_j2] = DQj2perturbation(M_D,mu,J2,r_Earth,r_I_DI,q_hat_DI,q_hat_BD);

% % Dual force summation
%f_hat_D = f_hat_D_g + f_hat_D_gradg + f_hat_D_j2;
f_hat_D = f_hat_D_g + f_hat_D_gradg;


%% Desired Frame Dynamics

omega_hat_dot_D_DI = DF_DYN_DQ(M_D,f_hat_D,omega_hat_D_DI);

q_hat_dot_DI = (1/2).*DQmult(q_hat_DI,omega_hat_D_DI);

states_dot(1:4,:) = omega_hat_dot_D_DI(:,1);
states_dot(5:8,:) = omega_hat_dot_D_DI(:,2);
states_dot(9:12,:) = q_hat_dot_DI(:,1);
states_dot(13:16,:) = q_hat_dot_DI(:,2);

%% Clean Up

if t == time_vec(:,invex)
    invex = invex + 1;

    r_I_DI_vec_data = [r_I_DI_vec_data;r_I_DI_vec'];
    
end

clc
fprintf('Simulation Time = %.2f sec\n',t);
fprintf('Completion Percentage = %.2f %%\n',(t/tmax)*100);

end