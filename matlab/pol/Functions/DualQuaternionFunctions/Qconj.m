function Product = Qconj(A)
% Quaternion Conjugate
% Inputs: (Quaternion)
%
% Dimensions: 
% Quaternion: 4x1

% Quaternion
A0 = A(1,:);      % Scaler part of quaternion
Avec = A(2:4,:);  % Vector part of quaternion

Product = [A0;-Avec];

end