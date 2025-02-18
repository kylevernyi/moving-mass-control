function [output] = SDQasterisk(M,a_check)

    M_1 = M(:,1:8);
    M_2 = M(:,9:16);

    a_hat_1 = a_check(1:4,:);
    a_hat_2 = a_check(5:8,:);

    output = DQmatrixmult(M_1,a_hat_1) + DQmatrixmult(M_2,a_hat_2);

end