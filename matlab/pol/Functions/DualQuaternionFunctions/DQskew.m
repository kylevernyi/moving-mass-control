function [q_hat_skew] = DQskew(q_hat)

    q_r = q_hat(:,1);
    
    q_d = q_hat(:,2);
    
    q_hat_skew = [skew(q_r),zeros(3,3);skew(q_d),skew(q_r)];

end