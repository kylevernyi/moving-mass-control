function Theta = AttitudeQuaternions2LinearizedEuler(q)

    % q: Attitude quaternion
    %
    % Theta = [roll;pitch;yaw]

    Theta = 2.*[(q(3,:)*q(4,:)) + (q(1,:)*q(2,:));
                (q(2,:)*q(4,:)) + (q(1,:)*q(3,:));
                (q(2,:)*q(3,:)) + (q(1,:)*q(4,:))];

end