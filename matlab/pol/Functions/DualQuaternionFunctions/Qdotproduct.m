function Product = Qdotproduct(A,B)
% Quaternion Dot Product
% Inputs: (Quaternion 1,Quaternion 2)
%
% Dimensions: 
% Quaternion 1: 4x1
% Quaternion 2: 4x1

% Quaternion 1
A0 = A(1,:);      % Scaler part of quaternion
Avec = A(2:4,:);  % Vector part of quaternion

% Quaternion 2
B0 = B(1,:);      % Scaler part of quaternion
Bvec = B(2:4,:);  % Vector part of quaternion

Product = (A0*B0) + dot(Avec,Bvec);

end