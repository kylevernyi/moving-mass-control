function Euler = Q2Euler(qvec_in)

% Function that converts a quaternion to Euler angles
% Reference: "https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles"
%
% Inputs: (Quaternion)
%
% Dimensions: 
% Quaternion: [4x1]
%
% NOTE: atan2() IS USED, AND NOT atan()

qvec = qvec_in./Qnorm(qvec_in); % Turning into unit quaternion

q0 = qvec(1,:);
q1 = qvec(2,:);
q2 = qvec(3,:);
q3 = qvec(4,:);

phi = atan2(2*((q0*q1) + (q2*q3)),1 - (2*((q1^2) + (q2^2))));

theta = asin(2*((q0*q2) - (q3*q1)));

psi = atan2(2*((q0*q3) + (q1*q2)),1 - (2*((q2^2) + (q3^2))));


Euler = [phi;theta;psi];

end