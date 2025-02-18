function x_dot = mbs3(t,u,t_max,J,M,m,g,theta,sigma,gamma,Kp,r_max)

global T delta_step Kps Rs

% definition of state space
omega = u(1:3);
q = u(4:7); q = q/norm(q);
theta_tilde = u(8:10);
beta_tilde = u(11:13);

e_delta = u(14:16);

% error definitions
e_u = omega - e_delta;

% estimation parameters definition
theta_hat = theta - theta_tilde;

% omega in quaternion multiplication form
omega_q = vector_quat_x(omega);
% rotation matrix from inertial to bff
R_bi = quat2rotmat(q);
% rotation of gravity vector in bff coordinates
g_b = R_bi*g;
% matrix form of gravity vector in bbf cordinates
g_b_x = vector_x(g_b);
% projection operator
Pp = (eye(3) - g_b*g_b'/(norm(g_b)^2));
% projection of e_u into e_u_p 
e_u_p = Pp*e_u;
% proportional gain definition
kp = Kp;
% phi definition ref[DOI: 10.2514/1.60380] eq(11)
phi = -M*g_b_x;
% commanded torque ref[DOI: 10.2514/1.60380] eq(20)
u_c = - phi*theta_hat - kp*e_u_p;
% maximum applicable torque
u_max = abs(r_max*m*g(3));

% limitation of u for torque saturation
u = u_c;
for k = 1:1:3
    if abs(u_c(k)) > u_max
        u(k) = u_max*sign(u_c(k));
    end
end

% torque after actuator saturation
delta_u = u - u_c;

r_com = cross(g_b,u)/(m*norm(g_b)^2);

if ismember(t,T) || mod(t,delta_step) ~= 0
else
    T = [T;t];
    Kps = [Kps;kp];
    Rs = [Rs;r_com'];
end

%e_u_dot = J\(-cross(e_u,J*e_u) + phi*theta + u_c+ delta_u.*beta_tilde);

e_delta_dot = J\(-cross(e_delta,J*e_delta) + beta_tilde.*delta_u);

%
omega_m_dot = J\(-cross(omega_m,J*omega_m) + phi*theta + u_c);

omega_dot = J\(-cross(omega,J*omega) + phi*theta + u);

q_dot = 0.5*omega_q*q; % quaternion derivative

theta_tilde_dot = - sigma*phi'*e_u;

beta_tilde_dot = - gamma*delta_u.*e_u;

x_dot = [omega_dot;q_dot;theta_tilde_dot;beta_tilde_dot;e_delta_dot];

% clc
% fprintf('Simulation Time = %.2f sec out of %.2f sec.\n',t,t_max);
% fprintf('Completion Percentage = %.2f %%\n',(t/t_max)*100);

end