function [q_hat_vert] = DQvert(q_hat)

% This function performs the dual quaternion ^| operation

    q_hat_vert = [q_hat(2:4,1);q_hat(2:4,2)];

end