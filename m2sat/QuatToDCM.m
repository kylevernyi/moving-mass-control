function [DCM] = QuatToDCM(q)
DCM = [zeros(3,3,length(q))];
for i=1:length(q)
    % create the direction cosine matrix from quaternion q
    C = [q(i,1)^2+q(i,2)^2-q(i,3)^2-q(i,4)^2, 2*(q(i,2)*q(i,3)+q(i,1)*q(i,4)), 2*(q(i,2)*q(i,4)-q(i,1)*q(i,3));
        2*(q(i,2)*q(i,3)-q(i,1)*q(i,4)), q(i,1)^2-q(i,2)^2+q(i,3)^2-q(i,4)^2, 2*(q(i,3)*q(i,4)+q(i,1)*q(i,2));
        2*(q(i,2)*q(i,4)+q(i,1)*q(i,3)), 2*(q(i,3)*q(i,4) - q(i,1)*q(i,2)), q(i,1)^2-q(i,2)^2-q(i,3)^2+q(i,4)^2];
    DCM(:,:,i) = C;%orth(C); % orthogonalizes columns via SVD (helps rounding errors)
end
end