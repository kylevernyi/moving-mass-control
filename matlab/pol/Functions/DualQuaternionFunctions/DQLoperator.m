function [q_hat_L] = DQLoperator(q_hat)

    q_r = q_hat(:,1);
    q_d = q_hat(:,2);

    q_hat_L = [QLoperator(q_r),zeros(4,3);QLoperator(q_d),QLoperator(q_r)];

end