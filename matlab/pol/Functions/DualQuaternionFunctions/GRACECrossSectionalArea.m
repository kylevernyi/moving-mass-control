function Ac = GRACECrossSectionalArea(v_vec_rel)

% This function calculates the cross-sectional area of GRACE. The main
% reference for the equations used in the function is "Study on Center of 
% Mass Calibration and K-band Ranging System Calibration of the GRACE 
% Mission" by Wang

%--------------------------------------------------------------------------
% Inputs
%
%     q: Attitude quaternions of spacecraft
%
% r_vec: ECIF position of spacecraft
%
% v_vec: ECIF velocity of spacecraft
% 
%--------------------------------------------------------------------------
% Outputs
%
%        Ac: Cross sectional area of GRACE
%
% v_vec_rel: Velocity of Spacecraft relative to Earth's atmosphere
% 
%--------------------------------------------------------------------------


% GRACE Constants (from GRACE-FO Handbook)

h = 0.721;   % [m] Height of GRACE
w_b = 1.944; % [m] Bottom width of GRACE
w_t = 0.695; % [m] Top width of GRACE
l = 3.1225;  % [m] Length of GRACE


% Calculating More Constants

c = (1/2)*(w_b - w_t);

a = sqrt((c^2) + (h^2));


% Surface Normal Vectors

n_1_hat = [1;0;0];
n_2_hat = [-1;0;0];

% n_3_hat = [0;sin(h/a);-cos(c/a)];
% n_4_hat = [0;-sin(h/a);-cos(c/a)];
n_3_hat = [0;h/a;-c/a];
n_4_hat = [0;-h/a;-c/a];

n_5_hat = [0;0;1];
n_6_hat = [0;0;-1];

n_i_hat = [n_1_hat,n_2_hat,n_3_hat,n_4_hat,n_5_hat,n_6_hat];


% Surface Areas

S_1 = (1/2)*h*(w_b + w_t);
S_2 = S_1;
% S_3 = l*h*(1/sin(h/a))
S_3 = a*l;
S_4 = S_3;
S_5 = w_b*l;
S_6 = w_t*l;

S_i = [S_1,S_2,S_3,S_4,S_5,S_6];


% Summation

Ac = 0;
Ac_vec = [];
for i = 1:6

    gamma = dot(n_i_hat(:,i),(v_vec_rel./norm(v_vec_rel)));

    if gamma < 0
        H = -1;
    else
        H = 0;
    end

    Ac_i = S_i(:,i)*dot(n_i_hat(:,i),(v_vec_rel./norm(v_vec_rel)))*H;

    Ac_vec = [Ac_vec,Ac_i]; % For debugging purposes only

    Ac = Ac + Ac_i;

end


end





