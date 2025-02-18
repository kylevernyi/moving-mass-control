function Product = DQcircleproduct(A,B)
% Dual Quaternion Circle Product
% Inputs: (Dual Quaternion 1,Dual Quaternion 2)
%
% Dimensions: 
% Dual Quaternion 1: [4x1,4x1]
% Dual Quaternion 2: [4x1,4x1]

% Dual Quaternion 1
Ar = A(:,1); % Real part of quaternion
Ad = A(:,2); % Dual part of quaternion

% Dual Quaternion 2
Br = B(:,1); % Real part of quaternion
Bd = B(:,2); % Dual part of quaternion

Product = Qdotproduct(Ar,Br) + Qdotproduct(Ad,Bd);

end