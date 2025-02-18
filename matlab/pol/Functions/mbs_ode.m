function x_dot = mbs_ode(t,u,J,M,m,g,r_off,sigma,kp)

global Rs

w1 = u(1); w2 = u(2); w3 = u(3);
q0 = u(4); q1 = u(5); q2 = u(6); q3 = u(7);
theta_hat1 = u(8); theta_hat2 = u(9); theta_hat3 = u(10);


omega = [w1;w2;w3];
q = [q0;q1;q2;q3]; q = q/norm(q);
theta_hat = [theta_hat1;theta_hat2;theta_hat3];

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

r_com_wn = r_com + var(0.5*10^-6*randn(1000,3))';

% real control with the dynamics of the system
tau = - m*cross(g_b,r_com_wn);



% Define first order ODE's
% q_dot
q_dot = 0.5*quat_mult(q,[0;omega]);
% Normalize quaternion
q_dot = q_dot/norm(q_dot);
% omega_dot
omega_dot = J\(- cross(omega,J*omega) + phi*r_off + tau_com);
% theta_hat_dot
theta_hat_dot = sigma*phi'*omega;


Rs = [Rs;r_com'];

x_dot = [omega_dot;q_dot;theta_hat_dot];


end