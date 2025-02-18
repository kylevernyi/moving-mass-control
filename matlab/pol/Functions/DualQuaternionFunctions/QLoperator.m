function [q_L] = QLoperator(q)

    q0 =  q(1,:);
    q_vec = q(2:4,:);

    q_L = [-q_vec';((q0.*eye(3,3)) + skew(q_vec))];

end