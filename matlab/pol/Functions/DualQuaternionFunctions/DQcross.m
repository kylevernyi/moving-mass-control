function Product = DQcross(A,B)
% Dual Quaternion Cross Product
% Inputs: (Dual Quaternion 1,Dual Quaternion 2)
%
% Dimensions: 
% Dual Quaternion 1: [4x1,4x1]
% Dual Quaternion 2: [4x1,4x1]

Product = (1/2).*(DQmult(A,B) - DQmult(DQconj(B),DQconj(A)));

end