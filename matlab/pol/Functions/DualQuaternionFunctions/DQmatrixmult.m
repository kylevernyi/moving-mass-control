function Product = DQmatrixmult(M,A)
% Dual Quaternion Matrix Multiplication
% Inputs: (Matrix, Dual Quaternion)
%
% Dimensions: 
% Matrix: 8x8
% Dual Quaternion: [4x1,4x1]

% Matrix
M11 = M(1:4,1:4);
M12 = M(1:4,5:8);
M21 = M(5:8,1:4);
M22 = M(5:8,5:8);

% Dual Quaternion
Ar = A(:,1); % Real part of quaternion
Ad = A(:,2); % Dual part of quaternion

Product = [Qmatrixmult(M11,Ar) + Qmatrixmult(M12,Ad),Qmatrixmult(M21,Ar) + Qmatrixmult(M22,Ad)];

end