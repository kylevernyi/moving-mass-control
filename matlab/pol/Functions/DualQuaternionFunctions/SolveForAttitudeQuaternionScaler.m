function q0 = SolveForAttitudeQuaternionScaler(q)

    q0 = sqrt(1 - (q(2,:)^2) - (q(3,:)^2) - (q(4,:)^2));

end