function Product = Qmatrixmult(M,A)
% Quaternion Matrix Multiplication
% Inputs: (Matrix, Quaternion)
%
% Dimensions: 
% Matrix: 4x4
% Quaternion: 4x1

% Matrix
M11 = M(1,1);
M12 = M(1,2:4);
M21 = M(2:4,1);
M22 = M(2:4,2:4);

% Quaternion
A0 = A(1,:);      % Scaler part of quaternion
Avec = A(2:4,:);  % Vector part of quaternion

Product = [(M11*A0) + (M12*Avec);(M21.*A0) + (M22*Avec)];

end