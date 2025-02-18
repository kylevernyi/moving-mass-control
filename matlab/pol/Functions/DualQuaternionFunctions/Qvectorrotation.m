function Aprime = Qvectorrotation(A,B)
% Quaternion Dot Product
% Inputs: (Quaternion 1,Quaternion 2)
%
% If A is a vector, input A as, [0;A]
%
%
%
% Dimensions: 
% Quaternion 1: 4x1 (quaternion to be rotated)
% Quaternion 2: 4x1 (the quaternion being rotated about)

% Turning B into a unit vector
% Bunit = B./Qnorm(B);

Bunit = B;

% TWO DIFFERENT ROTATION ORDERS:
% (Both correct)

%Aprime = Qmult(Qmult(Bunit,A),Qinv(Bunit));

Aprime = Qmult(Qmult(Bunit,A),Qconj(Bunit));

%Aprime = Qmult(Qmult(Qinv(Bunit),A),Bunit);

end