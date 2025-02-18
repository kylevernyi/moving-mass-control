function x_dot = mbs_ode_w_lfl(t,u,J,M,m,g,r_off,sigma,kp,sigma_f,gamma_f)

global Rs omega_ps

w1 = u(1); w2 = u(2); w3 = u(3);
q0 = u(4); q1 = u(5); q2 = u(6); q3 = u(7);
theta_hat1 = u(8); theta_hat2 = u(9); theta_hat3 = u(10);


omega = [w1;w2;w3];
q = [q0;q1;q2;q3]; q = q/norm(q);
theta_hat = [theta_hat1;theta_hat2;theta_hat3];
theta_hat_f = u(11:13);

% rotation of gravity vector in bff frame
g_b = quat_mult(quat_mult(quat_conj(q),[0;g]),q);
g_b = g_b(2:4);
% matrix form of gravity vector in bbf cordinates
g_b_x = vector_x(g_b);
% projection operator
Pp = (eye(3) - g_b*g_b'/(norm(g_b)^2));
% projection of omega into omega_p
omega_p = Pp*omega;
% phi definition ref[DOI: 10.2514/1.60380] eq(11) has a (-)
phi = -M*g_b_x;
% control torque ref[DOI: 10.2514/1.60380] eq(20)
tau_com = - phi*theta_hat - kp*omega;
%tau_com(3) = -kp*omega(3);

r_com = cross(g_b,tau_com)/(m*norm(g_b)^2);

% Define first order ODE's
% q_dot
q_dot = 0.5*quat_mult(q,[0;omega]);
% Normalize quaternion
q_dot = q_dot/norm(q_dot);
% omega_dot
omega_dot = J\(- cross(omega,J*omega) + phi*r_off + tau_com);
% theta_hat_dot
theta_hat_dot = sigma*(phi'*omega - sigma_f*(theta_hat - theta_hat_f));
% theta_hat_f_dot
theta_hat_f_dot = gamma_f*(theta_hat - theta_hat_f);

Rs = [Rs;r_com'];

omega_ps = [omega_ps;norm(omega_p)];

x_dot = [omega_dot;q_dot;theta_hat_dot;theta_hat_f_dot];


end