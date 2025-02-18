function T = Qrotationalmatrix(qvec_in)

    qvec = qvec_in./Qnorm(qvec_in); % Turning into unit quaternion

    q0 = qvec(1,:);
    q1 = qvec(2,:);
    q2 = qvec(3,:);
    q3 = qvec(4,:);

    T11 = (q0^2)+(q1^2)-(q2^2)-(q3^2);
    T12 = 2*((q1*q2)+(q0*q3));
    T13 = 2*((q1*q3)-(q0*q2));

    T21 = 2*((q1*q2)-(q0*q3));
    T22 = (q0^2)-(q1^2)+(q2^2)-(q3^2);
    T23 = 2*((q2*q3)+(q0*q1));

    T31 = 2*((q1*q3)+(q0*q2));
    T32 = 2*((q2*q3)-(q0*q1));
    T33 = (q0^2)-(q1^2)-(q2^2)+(q3^2);

    T = [T11,T12,T13;T21,T22,T23;T31,T32,T33];

end
