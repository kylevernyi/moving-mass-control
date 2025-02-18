function x_dot = mbs1(t,u,t_max,J,M,m,g,r_off,sigma,r_max,v_max,Kp)

global T delta_step Kps Rs tau1s tau2s tau_res q_norm

w1 = u(1); w2 = u(2); w3 = u(3);
q1 = u(4); q2 = u(5); q3 = u(6); q4 = u(7);
theta1 = u(8); theta2 = u(9); theta3 = u(10);
r1 = u(11); r2 = u(12); r3 = u(13);


omega = [w1;w2;w3];
q = [q1;q2;q3;q4]; q = q/norm(q);
theta = [theta1;theta2;theta3];
r = [r1;r2;r3];
%kp_t = u(14:16);
theta_e = r_off - theta;

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

% real control with the dynamics of the system
tau = - m*cross(g_b,r);

q_dot = 0.5*omega_q*q;
omega_dot = J\(- cross(omega,J*omega) + phi*r_off + tau_com); %%%%%
theta_dot = sigma*(phi'*omega);


q1_d = q_dot(1); q2_d = q_dot(2); q3_d = q_dot(3); q4_d = q_dot(4);



R_bi_dot = [                 -4*(q2*q2_d+q3*q3_d), 2*(q1_d*q2+q1*q2_d - q4_d*q3-q4*q3_d), 2*(q1_d*q3+q1*q3_d + q4_d*q2+q4*q2_d);
            2*(q1_d*q2+q1*q2_d + q4_d*q3+q4*q3_d),                  -4*(q1*q1_d+q3*q3_d), 2*(q2_d*q3+q2*q3_d - q4_d*q1+q4*q1_d);
            2*(q1_d*q3+q1*q3_d - q4_d*q2-q4*q2_d), 2*(q1_d*q3+q1*q3_d + q4_d*q1+q4*q1_d),                  -4*(q1*q1_d+q2*q2_d)];

g_b_dot = R_bi_dot*g;

g_b_x_dot = vector_x(g_b_dot);

phi_dot = -M*g_b_x_dot;

Pp_dot = - (g_b_dot*g_b' + g_b*g_b_dot') / (norm(g_b)^2);

omega_p_dot = Pp_dot*omega + Pp*omega_dot;

%kp_dot = - 2*kp*omega_p'*omega_p_dot;

tau_com_dot = - (phi_dot*theta + phi*theta_dot) - Kp*omega_p_dot;%(kp_dot*omega_p + kp*omega_p_dot);

% r = cross(g_b,tau_com)/(m*norm(g_b)^2)
% f_x = cross(g_b,tau_com);

f_x_dot = cross(g_b_dot,tau_com) + cross(g_b,tau_com_dot);


r_dot = f_x_dot / (m*norm(g_b)^2);

% for k = 1:1:3
%     if abs(r_dot(k)) > v_mass
%         r_dot(k) = -v_mass*sign(r_dot(k));
%     end
% end

x_dot = [omega_dot;q_dot;theta_dot;r_dot];

% clc
% fprintf('Simulation Time = %.2f sec out of %.2f sec.\n',t,t_max);
% fprintf('Completion Percentage = %.2f %%\n',(t/t_max)*100);

end