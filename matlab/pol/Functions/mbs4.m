function x_dot = mbs4(t,u,t_max,J,M,m,g,r_off,sigma,Kp,gamma_f,sigma_f)

global T delta_step Kps Rs tau1s tau2s tau_res q_norm


omega = u(1:3);
q = u(4:7); q = q/norm(q);
theta = u(8:10);
theta_f = u(11:13);

%theta_e = r_off - theta;

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
% projection of omega into omega_p 
omega_p = Pp*omega;
% phi definition ref[DOI: 10.2514/1.60380] eq(11)
phi = -M*g_b_x;
% control torque ref[DOI: 10.2514/1.60380] eq(20)
tau_com = - phi*theta - Kp*omega_p; %%%
%tau_com(3) = 0;

r_com = cross(g_b,tau_com)/(m*norm(g_b)^2);

if ismember(t,T) || mod(t,delta_step) ~= 0
else
    T = [T;t];
    Kps = [Kps;Kp];
    Rs = [Rs;r_com'];
    tau1s = [tau1s;(- phi*theta)'];
    tau2s = [tau2s;(- Kp*omega_p)'];
    tau_res = [tau_res;(phi*r_off + tau_com)'];
    q_norm = [q_norm;norm(q)];
end

q_dot = 0.5*omega_q*q;
omega_dot = J\(- cross(omega,J*omega) + phi*r_off + tau_com); %%%%%
theta_f_dot = gamma_f*(theta - theta_f);
theta_dot = sigma*(phi'*omega - sigma_f*(theta - theta_f));


x_dot = [omega_dot;q_dot;theta_dot;theta_f_dot];

% clc
% fprintf('Simulation Time = %.2f sec out of %.2f sec.\n',t,t_max);
% fprintf('Completion Percentage = %.2f %%\n',(t/t_max)*100);

end