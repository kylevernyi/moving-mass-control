function G = CalcMRP_Matrix(sigma)
    G = 0.5 * (eye(3) + skew(sigma) +sigma*sigma' - (0.5*(1+sigma'*sigma))*eye(3));

    % s = sigma'*sigma;
    % G = 0.25*((1-s)^2*eye(3) + 2*skew(sigma) + 2*sigma*sigma');
end