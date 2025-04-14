function [DCM] = QuatToDCM_single(q)
DCM = [zeros(3,3)];
    % create the direction cosine matrix from quaternion q
    C = [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)), 2*(q(2)*q(4)-q(1)*q(3));
        2*(q(2)*q(3)-q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*(q(3)*q(4)+q(1)*q(2));
        2*(q(2)*q(4)+q(1)*q(3)), 2*(q(3)*q(4) - q(1)*q(2)), q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    DCM= C;%orth(C); % orthogonalizes columns via SVD (helps rounding errors)
end