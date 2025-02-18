function [a,eccen,nu0,Omega,omega,incl] = OrbitalElementsFromRandV(r,v,mu)
%
% This function calcuates the orbital elements from positon and velocity as
% observed in the Earth Centered Inertial Frame
%
% The main reference for this code is, 
% [1] "Fundamentals of Astrodynamics" by Bate et.al.
%
%{ 
    a........................Semi-major axis
    eccen....................Eccentricity
    nu_time..................Time in relation to nu
    Omega....................Right ascention of the ascending node
    omega....................Argument of the perigee
    incl.....................Inclination
    mu.......................Standard gravitational parameter
%}

h = cross(r,v); % Angular momentum vector

n = cross([0;0;1],h); % Node vector

e_vec = (1/mu)*((((norm(v)^2) - (mu/norm(r))).*r) - (dot(r,v).*v));

p = (norm(h)^2)/mu;

eccen = norm(e_vec); % Eccentricity

incl = acosd(h(3,:)/norm(h));

% "Inclination is always less than 180 degees" [1]
if incl >= 180
    incl = incl - 360;
else
    incl = incl;
end

Omega = acosd(n(1,:)/norm(n));

% "If n_j > 0 then Omega is less than 180 degees" [1]
if n(2,:) > 0 && Omega >= 180
    Omega = Omega - 360;
else
    Omega = Omega;
end

omega = acosd(dot(n,e_vec)/(norm(n)*eccen));

% "If e_vec_k > 0 then omega is less than 180 degees" [1]
if e_vec(3,:) > 0 && omega >= 180
    omega = omega - 360;
else
    omega = omega;
end

nu0 = acosd(dot(e_vec,r)/(eccen*norm(r)));

% "If dot(r,v) > 0 then nu0 is less than 180 degees" [1]
if dot(r,v) > 0 && nu0 >= 180
    nu0 = nu0 - 360;
else
    nu0 = nu0;
end

% u0 = acosd(dot(n,r)/(norm(n)*norm(r)));
% 
% l0 = Omega + u0;


a = p/(1 - (eccen^2)); % Semi-minor axis


% Tau = (2*pi)*sqrt((a^3)/mu);  % Orbital period
% 
% % Finding the time of orbit related to nu
% 
% nu_time = (nu0*Tau)/(2*pi);


end