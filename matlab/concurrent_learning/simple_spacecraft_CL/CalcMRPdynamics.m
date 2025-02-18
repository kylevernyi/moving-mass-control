function [sigma_dot] = CalcMRPdynamics(sigma,omega)
    % sigma_dot = CalcMRP_Matrix(sigma)*omega;
    sigma_dot = 0.5 * (eye(3) * (1-norm(sigma))/2 + skew(sigma) + sigma*sigma') * omega;
end