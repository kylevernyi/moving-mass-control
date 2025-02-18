function [r_vec_ECIF,r_dot_vec_ECIF] = OrbitalElements2PositionVelocityECIF(a,e,nu,Omega,omega,i,mu)
%{ 

a........................Semi-major axis
e........................Eccentricity
nu.......................True anomaly
Omega....................Right ascention of the ascending node
omega....................Argument of the perigee
i........................Inclination
mu.......................Standard gravitational parameter

%}

p = a*(1 - (e^2)); % Semi-minor axis

%% In Perifocal Frame

r = p/(1 + (e*cosd(nu))); % Polar equation of a conic

r_vec_perifocal = [r*cosd(nu);r*sind(nu);0]; % Position of spacecraft

r_dot_vec_perifocal = sqrt(mu/p).*[-sind(nu);e + cosd(nu);0];

%% Transformation to Earth Centered Inertial Frame

Q = [];
Q(1,1) = (cosd(Omega)*cosd(omega)) - (sind(Omega)*sind(omega)*cosd(i));
Q(1,2) = (-cosd(Omega)*sind(omega)) - (sind(Omega)*cosd(omega)*cosd(i));
Q(1,3) = sind(Omega)*sind(i);
Q(2,1) = (sind(Omega)*cosd(omega)) + (cosd(Omega)*sind(omega)*cosd(i));
Q(2,2) = (-sind(Omega)*sind(omega)) + (cosd(Omega)*cosd(omega)*cosd(i));
Q(2,3) = -cosd(Omega)*sind(i);
Q(3,1) = sind(omega)*sind(i);
Q(3,2) = cosd(omega)*sind(i);
Q(3,3) = cosd(i);

r_vec_ECIF = Q*r_vec_perifocal;

r_dot_vec_ECIF = Q*r_dot_vec_perifocal;

end