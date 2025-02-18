function Product = DQconj(A)
% Dual Quaternion Conjugate
% Inputs: (Quaternion)
%
% Dimensions: 
% Dual Quaternion: [4x1,4x1]

% Dual Quaternion
Ar = A(:,1); % Real part of quaternion
Ad = A(:,2); % Dual part of quaternion

Product = [Qconj(Ar),Qconj(Ad)];

end