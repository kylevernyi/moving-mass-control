
%% Dynamics
function [xdot_return] = mrac_dynamics(t, x, A, B, Ar, Br, P, ...
    gamma_x, gamma_R, CL_point_accept_epsilon, p_bar, CL_on, CL_K_gain, CL_Kr_gain, Kr_star, K_star, ...
    Q_cov, R_cov, Phi_stm, H, v_smooth, ...
    integration_phase)
    global point_added min_sv;
    global u_global r_global  V_global nu_global ; % for plotting
    %% Persistent variables keep their values between function calls for use next time
    persistent X R X_dot; % CL variables
    persistent x_last_point_stored p current_index_for_cyclic_stack; % CL variables
    persistent nu L P_kf Pi Sigma lambda j_smooth iter xdot_hat_available ; %  Kalman filter variables
    persistent r;

    %% Unpack states 
    x = reshape(x, [length(x) 1]);
    xr = x(3:4);
    K = x(5:6);
    Kr = x(7);
    x = x(1:2);

    % first time initialization of persistent variables
    if isempty(iter)
        j_smooth = 1; % start FPS after this many points (maybe wait a few so kalman converges first?)
        iter = 1  % keeps track of which iteration we are on 
        nu = x;
        P_kf = [1*eye(1), zeros(1,1); 
                 zeros(1,1),  1*eye(1)]; % initial covariance matrix 
    end
    

    %% Reference signal
    if (t < 10)  
        r = 5;
    elseif (t >= 10 && t <= 20) 
        r = -5;
    else 
        % r = 5;
    end

    %% Kalman filter + smoother
if integration_phase
    y = H*x + sqrtm(R_cov)*randn(1,1); % measurement

    if (iter < j_smooth)  % regular kalman filter until we turn on fps
        L(:,iter) = (Phi_stm*P_kf(:,:,iter)*H') * inv(H*P_kf(:,:,iter)*H' + R_cov);
        nu(:,iter+1) = Phi_stm*nu(:,iter) + L(:,iter)*(y - H*nu(:,iter));
        P_kf(:,:, iter+1) = Phi_stm*P_kf(:,:,iter) * (Phi_stm - L(:,iter)*H)' + Q_cov;
        xdot_hat_available = true;
    elseif (iter >= j_smooth) % we need to at least be to the jth point to start smoothing
        % Fixed point smoother initialization
        Sigma(:,:,j_smooth) = P_kf(:,:,j_smooth);
        Pi(:,:,j_smooth) = P_kf(:,:,j_smooth);

        for k = j_smooth:1:iter % iterate from oldest point to smooth (j_smooth) to current measurement (iter)
            L(:,k) = (Phi_stm*P_kf(:,:,k)*H') /(H*P_kf(:,:,k)*H' + R_cov); % forward gain
            lambda(:, k) = (Sigma(:,:,k)*H') / (H*P_kf(:,:,k)*H' + R_cov); % smoother kalman gain
            
            nu(:, j_smooth) = nu(:, j_smooth) + lambda(:,k) * (y - H*nu(:, k)); % updating previous estimates here, note j_smooth

            nu(:,k+1) = Phi_stm*nu(:,k) + L(:,k)*(y - H*nu(:,k)); % forward update
            P_kf(:,:, k+1) = Phi_stm*P_kf(:,:,k) * (Phi_stm - L(:,k)*H)' + Q_cov; % forward covariance update

            % Propagate covariance (Pi) and cross variance (sigma). 
            % We mostly care to inspect Pi. sigma is an intermediate variable
            Pi(:,:,k+1) = Pi(:,:,k) - Sigma(:,:,k)*H'*lambda(:,k)'; % backwards update
            Sigma(:,:,k+1) = Sigma(:,:,k)*(Phi_stm - L(:,k)*H)'; % backwards update

            % enforce symmetry (optional but maybe useful in real life with numerical issues)
            % Sigma(:,:,k+1) = 0.5 * (Sigma(:,:,k+1) + Sigma(:,:,k+1)');
            % Pi(:,:,k+1) = 0.5 * (Pi(:,:,k+1) + Pi(:,:,k+1)');
        end
        xdot_hat_available = true;
    end

    % if current point is more than "v_smooth" points away from the oldest smoothed point  
    % we move our oldest point up to smooth the next point
    if ( (iter+1) - j_smooth ) > v_smooth
        j_smooth = j_smooth+1;
    end
end

    %% Control
    e = nu(:,iter) - xr; % tracking error signal
    % e = x - xr;
    K_tilde = K - K_star; % estimation errors (used for analysis not controller)
    Kr_tilde = Kr - Kr_star; 


    u_pd = K'  * nu(:,iter);
    u_rm = Kr' * r; % feed forward part of control signal
    u = u_pd + u_rm; % control signal



    
    %% Concurrent learning data selection algorithm
    if (isempty(x_last_point_stored)) % initialize if we are just starting
        x_last_point_stored = nu(:,iter);
        p = 0; current_index_for_cyclic_stack = 1;
    end
    
    if (integration_phase && CL_on) % only do this if integrating not backsolving since global vars get messed up 
        if (xdot_hat_available)
    
            xhat_i = nu(1,iter); % x estimate
            xdot_hat_i = nu(2,iter); % xdot estimate
        
            if (  ( norm(nu(:,iter) - x_last_point_stored)^2 / norm(nu(:,iter)) >= CL_point_accept_epsilon)  || (rank([X, nu(:,iter)]) > rank(X))) 
                if (p < p_bar) % Initilization, record more data until p_bar points
                    p = p+1;
                    current_index_for_cyclic_stack = p;
                    X(:, p) = nu(:,iter);
                    R(:, p) = r;
                    x_last_point_stored = nu(:,iter);
                    point_added = [point_added t];
                else
        
                    % Cyclic history stack
                    [X, ~] = updateHistoryStack(X, current_index_for_cyclic_stack, xhat_i, p_bar);
                    [R, ~] = updateHistoryStack(R, current_index_for_cyclic_stack, r, p_bar);
                    [X_dot, current_index_for_cyclic_stack] = updateHistoryStack(X_dot, current_index_for_cyclic_stack, xdot_hat_i, p_bar);
                    point_added = [point_added t];

                    % MinSVD Maximization Algorithm (activates after p_bar points)
                    % Now we only add new points if it makes our minimum singular value (msv) increase
                %     T = X; % temp storage of X
                %     S_min_old = min(svd(X, "vector")); % get the current minimum singular values
                % 
                %     S = zeros([p 1]); % preallocate to hold all new min sv that we are going to test
                %     for j = 1:p 
                %         % iterate through each column in X and replace it with the new data point. 
                %         % calculate and store the new minimum singular value for comparison later
                %         X(:,j) = nu(:,iter);
                %         S(j) = min(svd(X,"vector")); %store calculated min sing value
                %         X = T; % restore X back to its original value for the next iteration
                %     end
                % 
                %     [max_S, max_S_idx] = max(S); 
                %     if (max_S > S_min_old) % if any new combinations increase our minimum SV
                %         X(:,max_S_idx) = nu(:,iter); % store new x if we have increased our SV
                %         R(:,max_S_idx) = r;
                %         x_last_point_stored = nu(:,iter);
                %         point_added = [point_added t];
                %     end
                end 
                
            end % sufficient change or adding data check end
        end % nu available end
    end

    %% Concurrent learning error signals 
    concurrent_learning_x = 0; concurrent_learning_R = 0;  % initialize sum to zero
    e_Kr = zeros([p 1]); e_K = zeros([p 2]);
    for j = 1:p
        x_j = X(:,j); % jth state vector from storage
        xdot_j = x_j(2);
        r_j = R(:,j); % jth reference signal from storage

        e_Kr(j) = Kr'*r_j - pinv(B)*Br*r_j;

        e_K(j, :) = pinv(B) * (xdot_j  - Ar*x_j(1) - Br*r_j - B*e_Kr(j));

        concurrent_learning_x = concurrent_learning_x + x_j*e_K(j)'; 
        concurrent_learning_R = concurrent_learning_R + r_j*e_Kr(j)'; 
    end

    %% Dynamics and update laws
    K_dot = -gamma_x * (nu(:,iter)*e'*P*B + CL_on*CL_K_gain*concurrent_learning_x);
    Kr_dot = -gamma_R * (r*e'*P*B + CL_on*CL_Kr_gain*concurrent_learning_R);

    xdot = A*x + B*u + sqrtm(Q_cov)*randn(2,1)*0;
    xr_dot = Ar*xr + Br*r;
       
    V = 0.5*e'*P*e + 0.5*trace(K_tilde'*inv(gamma_x)*K_tilde) + 0.5*trace(Kr_tilde'*inv(gamma_R)*Kr_tilde); % lyap function

    xdot_return = [xdot; xr_dot; K_dot; Kr_dot];
    u_global(:, iter) = u;
    r_global(:,iter) = r;
    V_global(:,iter) = V;
    nu_global(:,iter) = nu(:,iter);

    iter = iter + 1;

    % Record the MinSVD for plotting
    if (integration_phase)
        min_svd = min(svd(X,"vector"));
        if isempty(min_svd)
            min_sv = [min_sv 0];
        else
            min_sv =  [min_sv min_svd];
        end
    end

    % helper for cyclic stack
    function [history, currentIndex] = updateHistoryStack(history, currentIndex, newElement, p_bar)
        history(:,currentIndex) = newElement;    % Insert the new element at the current index
        currentIndex = mod(currentIndex, p_bar) + 1;     % Update the current index in a cyclic manner
    end
end
