function Result = Qrotate(A,B)
% Quaternion Rotation
% Inputs: (Quaternion 1,Quaternion 2)
%
% Dimensions: 
% Quaternion 1: 4x1 (quaternion getting rotated)
% Quaternion 2: 4x1 (quaternion doing to rotation)

Bunit = B./Qnorm(B);

Result = Qmult(Bunit,Qmult(A,Qconj(Bunit)));

end