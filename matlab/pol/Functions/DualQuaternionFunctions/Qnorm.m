function Result = Qnorm(A)
% Quaternion Norm
% Inputs: (Quaternion)
%
% Dimensions: 
% Quaternion: 4x1

% Quaternion
A0 = A(1,:);   % Scaler part of quaternion
A1 = A(2,:);   % 2nd element of quaternion
A2 = A(3,:);   % 3rd element of quaternion
A3 = A(4,:);   % 4th element of quaternion

Result = sqrt((A0^2) + (A1^2) + (A2^2) + (A3^2));

% Result = sqrt(Qdotproduct(A,A));

end