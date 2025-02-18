function [q_hat_dot_tilde_TMB,b_hat_dot_tilde_omega,P_dot,q_hat_tilde_TMB_plus,b_hat_tilde_omega_plus,P_plus,omega_hat_tilde_TM_TMB] = DQFilipeKalmanFilterOriginal(q_hat_TMB_measured,q_hat_tilde_TMB,b_hat_tilde_omega,P,Q_prime,R)



    %% Measurement Update

    % Step 1

    H = [DQLoperator(q_hat_tilde_TMB),zeros(8,6)];

    % Step 2

    K = P*((H')*inv(H*(P*(H')) + R));

    % Step 3

%     q_hat_TMB_measured = [q_TMB_measured,(1/2).*Qmult(r_B_TMB_measured,q_TMB_measured)];

    % Step 4

    z = DQvertOriginal(q_hat_TMB_measured);

    z_tilde = DQvertOriginal(q_hat_tilde_TMB);

    Deltas = K*(z - z_tilde);

    Delta_delta_q_bar_tilde_TMB = Deltas(1:6,:);

    Delta_b_bar_tilde_omega = Deltas(7:12,:);

    % Step 5

    qr = Delta_delta_q_bar_tilde_TMB(1:3,:);

    qd = Delta_delta_q_bar_tilde_TMB(4:6,:);

    if norm(qr) <= 1

        c = sqrt(1 - (norm(qr)^2));

        Delta_delta_q_hat_tilde_TMB = [[c;qr],[-(1/c).*((qr')*(qd));qd]];

    elseif norm(qr) > 1

        d = sqrt(1 + (norm(qr)^2));

        Delta_delta_q_hat_tilde_TMB = [[1/d;(1/d).*qr],[-d.*((qr')*(qd));qd]];

    end


    % Step 6

    q_hat_tilde_TMB_plus = DQmult(q_hat_tilde_TMB,Delta_delta_q_hat_tilde_TMB);

    b_bar_tilde_omega = [b_hat_tilde_omega(2:4,1);b_hat_tilde_omega(2:4,2)];

    b_bar_tilde_omega = b_bar_tilde_omega + Delta_b_bar_tilde_omega;

    b_hat_tilde_omega_plus = [[0;b_bar_tilde_omega(1:3,:)],[0;b_bar_tilde_omega(4:6,:)]];

    % Step 7

    PPP = eye(12,12) - (K*H);

    P_plus = (PPP*(P*(PPP'))) + (K*(R*(K')));

    %% Time Update

    % Step 1

    q_tilde_TMB = q_hat_tilde_TMB_plus(:,1);

%     q_tilde_TMB = q_tilde_TMB./Qnorm(q_tilde_TMB); % Normalize attitude quaternion

    r_tilde_B_TMB = 2.*Qmult(q_hat_tilde_TMB_plus(:,2),Qconj(q_hat_tilde_TMB_plus(:,1)));
%     r_tilde_B_TMB_vec = r_tilde_B_TMB(2:4,:);
    
    q_hat_tilde_TMB = [q_tilde_TMB,(1/2).*Qmult(r_tilde_B_TMB,q_tilde_TMB)];
%     q_hat_tilde_TMB = [q_tilde_TMB,(1/2).*Qmult([0;r_tilde_B_TMB_vec],q_tilde_TMB)];

    % Step 2 if a new measurement update is avaliable go to the measurement
    % update

    % Step 3 measurement of dual velocity is set to zero

    % Step 4

    omega_hat_tilde_TM_TMB = -b_hat_tilde_omega_plus;

    % Step 5 output q_hat_tilde_TMB and omega_hat_tilde_TM_TMB

    % Step 6

    F = [-DQskew(omega_hat_tilde_TM_TMB),-(1/2).*eye(6,6);zeros(6,6),zeros(6,6)];

    G = [-(1/2).*eye(6,6),zeros(6,6);zeros(6,6),eye(6,6)];

    % Step 7

    q_hat_dot_tilde_TMB = (1/2).*DQmult(q_hat_tilde_TMB,omega_hat_tilde_TM_TMB);

    Q = [zeros(6,6),zeros(6,6);zeros(6,6),Q_prime];

    P_dot = F*P_plus + P_plus*(F') + G*(Q*G');

    % Step 8

    %%%%%%% THIS PART %%%%%%%%

%     b_hat_dot_omega = [[0;eta_bar_b_omega],[0;eta_bar_b_v]];
% 
%     b_hat_dot_tilde_omega = E(b_hat_dot_omega);

    b_hat_dot_tilde_omega = zeros(4,2); % Because the guassian white noise on the measurements is assumed to be zero mean


end