function [qdes,dcm] = DCM2quat_LVLH(r,v,qdesold)

%h = orbit's angular momentum
%orb = 6 component vector [r,v] of the S/C

q = [1; 0; 0; 0]; %initializing
qdes = [1; 0; 0; 0]; %initializing

h = cross(r,v);

a3 = -r/norm(r);
a2 = -h/norm(h);
a1 = cross(a2, a3);

%direction cosine matrix between LVLH and ECI
dcm = [a1, a2, a3]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%transforming into quaternion (modified Matlab function)
q(4) =  0; 

tr = trace(dcm);

if (tr > 0)
    sqtrp1 = sqrt( tr + 1.0 );

    q(1) = 0.5*sqtrp1; 
    q(2) = (dcm(2, 3) - dcm(3, 2))/(2.0*sqtrp1);
    q(3) = (dcm(3, 1) - dcm(1, 3))/(2.0*sqtrp1); 
    q(4) = (dcm(1, 2) - dcm(2, 1))/(2.0*sqtrp1); 
else
    d = diag(dcm);
    if ((d(2) > d(1)) && (d(2) > d(3)))
        % max value at dcm(2,2,i)
        sqdip1 = sqrt(d(2) - d(1) - d(3) + 1.0 );

        q(3) = 0.5*sqdip1; 

        if ( sqdip1 ~= 0 )
            sqdip1 = 0.5/sqdip1;
        end

        q(1) = (dcm(3, 1) - dcm(1, 3))*sqdip1; 
        q(2) = (dcm(1, 2) + dcm(2, 1))*sqdip1; 
        q(4) = (dcm(2, 3) + dcm(3, 2))*sqdip1; 
    elseif (d(3) > d(1))
        % max value at dcm(3,3,i)
        sqdip1 = sqrt(d(3) - d(1) - d(2) + 1.0 );

        q(4) = 0.5*sqdip1; 

        if ( sqdip1 ~= 0 )
            sqdip1 = 0.5/sqdip1;
        end

        q(1) = (dcm(1, 2) - dcm(2, 1))*sqdip1;
        q(2) = (dcm(3, 1) + dcm(1, 3))*sqdip1; 
        q(3) = (dcm(2, 3) + dcm(3, 2))*sqdip1; 
    else
        % max value at dcm(1,1,i)
        sqdip1 = sqrt(d(1) - d(2) - d(3) + 1.0 );

        q(2) = 0.5*sqdip1; 

        if ( sqdip1 ~= 0 )
            sqdip1 = 0.5/sqdip1;
        end

        q(1) = (dcm(2, 3) - dcm(3, 2))*sqdip1; 
        q(3) = (dcm(1, 2) + dcm(2, 1))*sqdip1; 
        q(4) = (dcm(3, 1) + dcm(1, 3))*sqdip1; 
    end
end
%%%%%%%%%MODIFIED FROM MATLAB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%reordering to match our definition of quaternion
% qdes = [q(4); q(1); q(2); q(3)];

if dot(qdesold, qdes) < 0
    qdes = -qdes;
end