function G = CalcMRP_Matrix(sigma)
    G = 0.5 * (eye(3) + skew(sigma) +sigma*sigma' - (0.5*(1+sigma'*sigma))*eye(3));
end