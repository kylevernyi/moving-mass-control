%% Fixed point smoother
% The smoothed estimate of x is always better than the standard 
% Kalman filter estimate (proof in textbook). 
% The larger the value of v (i.e., the more measurements that we use
% to obtain our smoothed estimate), the greater the improvement 
% in the estimation accuracy. 
% The better our sensors, the better the smoother performs
% We expect that Pi will converge to P in steady state
clc; clear all; close all;
rng(1); % set the random number generator to be the same every time for consistency

tf = 50; % sim time parameters
dt = 0.01;
t = 0:dt:tf;

% Noise params
R = 0.001;
Q = 0.01*eye(2);

% System matrices
F = expm([0 1; 0 0]*dt); % state transition matrix
H = [1 0]; % measurement matrix (measure position)
G = [0; 1]; % control matrix

% Fixed point config how many point we wish to smooth.
% ie use the next v measurements to smooth point j
v = 150; 

% initial conditions
x = [1; 1];
xhat = [1;1]; % make xhat slightly wrong
P = .01 *[1 0; 0 1]; % covariance matrix
xhat_original = xhat; % store regular kalman filter answer here

j = 10; % start FPS after this many points (maybe wait a few so kalman converges first?)
   
for i = 1:length(t)-1
    % Measurement of current state
    y(i) = H*x(:, i) + sqrtm(R)*randn(1,1);
    
    % Regular apriori kalman filter runs every time step
    if (i < j)  % regular kalman filter until we turn on fps
        L(:,i) = (F*P(:,:,i)*H') /(H*P(:,:,i)*H' + R);
        xhat(:,i+1) = F*xhat(:,i) + L(:,i)*(y(i) - H*xhat(:,i));
        P(:,:, i+1) = F*P(:,:,i) * (F - L(:,i)*H)' + Q;
        xhat_original(:,i+1) =  xhat(:,i+1); % save for comparison
    elseif (i >= j) % we need to at least be to the jth point to start smoothing
        % Fixed point smoother initialization
        Sigma(:,:,j) = P(:,:,j);
        Pi(:,:,j) = P(:,:,j);

        for k = j:1:i % iterate from oldest point to smooth (j) to current measurement (i)
            L(:,k) = (F*P(:,:,k)*H') /(H*P(:,:,k)*H' + R);
            lambda(:, k) = (Sigma(:,:,k)*H') / (H*P(:,:,k)*H' + R); % smoother kalman gain
            
            xhat(:, j) = xhat(:,j) + lambda(:,k) * (y(k) - H*xhat(:,k)); % updating previous estimates here, note j

            xhat(:,k+1) = F*xhat(:,k) + L(:,k)*(y(k) - H*xhat(:,k));
            P(:,:, k+1) = F*P(:,:,k) * (F - L(:,k)*H)' + Q;

            % Propagate covariance (Pi) and cross variance (sigma). 
            % We mostly care to inspect Pi. sigma is an intermediate variable
            Pi(:,:,k+1) = Pi(:,:,k) - Sigma(:,:,k)*H'*lambda(:,k)';
            Sigma(:,:,k+1) = Sigma(:,:,k)*(F - L(:,k)*H)';

            % enforce symmetry
            Sigma(:,:,k+1) = 0.5 * (Sigma(:,:,k+1) + Sigma(:,:,k+1)');
            Pi(:,:,k+1) = 0.5 * (Pi(:,:,k+1) + Pi(:,:,k+1)');

                xhat_original(:,k+1) =  xhat(:,k+1); % save for comparison

        end
    end


    % if current point is more than "v" points away from the oldest smoothed point  
    % we move our oldest point up to smooth the next point
    if ( (i+1) - j ) > v
        j = j+1;
    end

    % Dynamics
    u = -.1*x(2, i) + .1*cos(.05*t(i));
    x(:,i+1) = F*x(:, i)+ G*u  ;%+ 0.001* sqrtm(Q)*randn(2,1); % true state  % 
   
end
% at this point, xhat has been smoothed while xhat_original has not

% Error calculations
e_kalman = x - xhat_original;
e_smoothed = x - xhat;
e_meas = x(1,2:end) - y;


figure; 
plot(t, x(1,:), t, xhat(1,:), t, xhat_original(1,:))
ylabel("Position"); title("Position Estimation");
xlabel("Time (sec)");
legend(["True" "Smoothed" "Kalman"]);

figure; 
plot(t, x(2,:), t, xhat_original(2,:),  t, xhat(2,:))
ylabel("Velocity"); title("Velocity Estimation");
xlabel("Time (sec)");
legend(["True"  "Kalman" "Smoothed"]);



P1 = squeeze(P(1,1,:));
P2 = squeeze(P(2,2,:));
Pi1 = squeeze(Pi(1,1,:));
Pi2 = squeeze(Pi(2,2,:));

figure;
plot(t, P1, t, P2, t, Pi1, "--", t, Pi2, "--");
title("Covariance vs Time"); xlabel("Time (sec)");
ylabel("Covariance"); 
legend(["P1" "P2" "Pi1" "Pi2"])

figure; 
plot(t, [e_meas 0], t, e_kalman(1,:), t, e_smoothed(1,:) );
title("Estimation Error Signals")
legend(["Meas" "Estimated" "Smoothed"]);


figure; 
plot(t(2:end), L)
title("Kalman Gains over Time"); xlabel("Time (sec)")

trP = [];
trPi = [];
for q = 1:length(t)
    trP(q) = trace(P(:,:,q));
    trPi(q) = trace(Pi(:,:,q));
end
figure; hold on;
plot(t, trP, t, trPi);
title("Tr(Cov)"); ylabel("Tr(cov)"); xlabel("Time (sec)");
legend(["P" "Pi"])

figure; hold on;
plot(t(2:end), lambda); title("Smoother Gains");
ylabel("\lambda"); xlabel("Time (sec)"); legend(["\lambda_1" "\lambda_2"])

fprintf("Position Error Norm:\n");
fprintf("Meas: %f   Kalman: %f  Smoothed:%f\n\n", norm(e_meas(1,:)),  norm(e_kalman(1,:)), norm(e_smoothed(1,:)));



fprintf("Velocity Error Norm:\n");
fprintf("Kalman: %f  Smoothed:%f\n\n",  norm(e_kalman(2,:)), norm(e_smoothed(2,:)));
