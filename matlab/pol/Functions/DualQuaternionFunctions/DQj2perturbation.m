function  [f_hat_B_j2,a_hat_B_j2] = DQj2perturbation(M_B,mu,J2,Re,r_I_BI,q_hat_BI)

% Function for calculating the dual force due to J2 perturbation effect
% 
% %constants of the problem START%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% G = 6.6742*10^(-20); %m^3/(kg*s^2) gravitational constant
% m1 = 5.972*10^24; %kg mass of Earth
% mu = G*m1; %km^3/s^2 gravitational parameter
% J2 = 0.00108263; % [-] second zonal harmonic
% Re = 6378; %km Earth's radius
% %constants of the problem END%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Getting components of linear position vector
r_I_BI_vec = r_I_BI(2:4,:);
% r_I_BI_vec = r_I_BI(2:4,:)./1000;
r_I_BI_x = r_I_BI_vec(1,:);
r_I_BI_y = r_I_BI_vec(2,:);
r_I_BI_z = r_I_BI_vec(3,:);

element1 = r_I_BI_x - ((5*r_I_BI_x*(r_I_BI_z^2))/(norm(r_I_BI_vec)^2));

element2 = r_I_BI_y - ((5*r_I_BI_y*(r_I_BI_z^2))/(norm(r_I_BI_vec)^2));

element3 = (3*r_I_BI_z) - ((5*(r_I_BI_z^3))/(norm(r_I_BI_vec)^2));

a_I_j2_vec = -((3/2)*((mu*J2*(Re^2))/(norm(r_I_BI_vec)^5))).*[element1;element2;element3]; % Gravitational acceleration vector

a_hat_I_j2 = [[0;a_I_j2_vec],zeros(4,1)];

a_hat_B_j2 = DQswap(DQmult(DQconj(q_hat_BI),DQmult(DQswap(a_hat_I_j2),q_hat_BI)));

% f_hat_B_j2 = DQmatrixmult(M_B,a_hat_B_j2).*1000;
f_hat_B_j2 = DQmatrixmult(M_B,a_hat_B_j2);


end