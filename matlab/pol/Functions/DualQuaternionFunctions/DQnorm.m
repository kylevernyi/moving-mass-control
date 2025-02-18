function result = DQnorm(A)
% Dual Quaternion Norm
% Inputs: (Dual Quaternion)
%
% Dimensions: 
% Dual Quaternion: [4x1,4x1]

result = sqrt(DQcircleproduct(A,A));

end