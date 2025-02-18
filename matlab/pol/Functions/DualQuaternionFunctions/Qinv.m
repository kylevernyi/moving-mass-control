function Inverse = Qinv(A)
% Quaternion Inverse
% Inputs: (Quaternion)
%
% Dimensions: 
% Quaternion: 4x1

Inverse = Qconj(A)./(Qnorm(A)^2);

end