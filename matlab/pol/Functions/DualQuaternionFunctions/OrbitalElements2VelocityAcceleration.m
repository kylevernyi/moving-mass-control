function [r_vec,r_dot_vec,Omega_vec,r_ddot_vec,Omega_dot_vec,Tau] = OrbitalElements2VelocityAcceleration(a,eccen,t,nu0,Omega,omega,incl,mu)
%{ 

a........................Semi-major axis
eccen....................Eccentricity
t........................Time
nu.......................True Anomaly
nu_or_t..................Set to 1 if using nu, anything else for using t
Omega....................Right ascention of the ascending node
omega....................Argument of the perigee
incl.....................Inclination
mu.......................Standard gravitational parameter

%}


%Finding the True Anomaly (nu)

if eccen == 0 % Circular Orbit

    Tau = (2*pi)*sqrt((a^3)/mu);  % Orbital period

    tinitial = (nu0/(2*pi))*Tau;% Time since periapsis

    time = t + tinitial;

    nu = ((2*pi)/Tau)*time; % True Anomaly

else % Elliptical Orbit

    Tau = ((2*pi)/sqrt(mu))*(a^(3/2));  % Orbital period

    % Finding initial time since periapsis

    E0 = 2*atand(sqrt((1 - eccen)/(1 + eccen))*tand(nu0/2));

    Me0 = E0 - (eccen*sind(E0));

    tinitial = (Me0/(2*pi))*Tau; % Time since periapsis

    time = t + tinitial;

    Me = (2*pi*time)/Tau; % Mean anomaly

    Eterm1 = eccen*sind(Me);
    Eterm2 = ((eccen^2)/2)*sind(2*Me);
    Eterm3 = ((eccen^3)/8)*((3*sind(3*Me)) - sind(Me));

    E = Me + Eterm1 + Eterm2 + Eterm3; % Eccentric Anomaly

    nu = acosd((eccen - cosd(E))/((eccen*cosd(E)) - 1));

end

%--------------------------------------------------------------------------
% Finding the True Anomaly (nu)

    Tau = ((2*pi)/sqrt(mu))*(a^(3/2));  % Orbital period

    % Finding initial time since periapsis

    E0 = 2*atand(sqrt((1 - eccen)/(1 + eccen))*tand(nu0/2));

    Me0 = E0 - (eccen*sind(E0));

    tinitial = (Me0/(2*pi))*Tau; % Time since periapsis

    time = t + tinitial;

    Me = (2*pi*time)/Tau; % Mean anomaly

    Eterm1 = eccen*sind(Me);
    Eterm2 = ((eccen^2)/2)*sind(2*Me);
    Eterm3 = ((eccen^3)/8)*((3*sind(3*Me)) - sind(Me));

    E = Me + Eterm1 + Eterm2 + Eterm3; % Eccentric Anomaly

    nu = acosd((eccen - cosd(E))/((eccen*cosd(E)) - 1));

%--------------------------------------------------------------------------



p = a*(1 - (eccen^2)); % Semi-minor axis

% In Perifocal Frame

r = p/(1 + (eccen*cos(nu))); % Polar equation of a conic

r_vec_perifocal = [r*cos(nu);r*sin(nu);0]; % Position of spacecraft

r_dot_vec_perifocal = sqrt(mu/p).*[-sin(nu);eccen + cos(nu);0];

% Transformation to Earth Centered Inertial Frame

Q = [];
Q(1,1) = (cosd(Omega)*cosd(omega)) - (sind(Omega)*sind(omega)*cosd(incl));
Q(1,2) = (-cosd(Omega)*sind(omega)) - (sind(Omega)*cosd(omega)*cosd(incl));
Q(1,3) = sind(Omega)*sind(incl);
Q(2,1) = (sind(Omega)*cosd(omega)) + (cosd(Omega)*sind(omega)*cosd(incl));
Q(2,2) = (-sind(Omega)*sind(omega)) + (cosd(Omega)*cosd(omega)*cosd(incl));
Q(2,3) = -cosd(Omega)*sind(incl);
Q(3,1) = sind(omega)*sind(incl);
Q(3,2) = cosd(omega)*sind(incl);
Q(3,3) = cosd(incl);

% Valuues in the ECIF

r_vec = Q*r_vec_perifocal;

r_dot_vec = Q*r_dot_vec_perifocal; % Linear velocity vector

% Velocities and Accelerations

r_ddot_vec = -(mu/(r^3)).*r_vec; % Linear acceleration vector

h_vec = cross(r_vec,r_dot_vec);

Omega_vec = h_vec/(r^2); % Angular velocity vector

Omega_dot_vec = -2*(dot(r_dot_vec,r_vec)/(r^2))*Omega_vec; % Angular acceleration vector

end