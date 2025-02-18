function [a_sun,a_lunar] = Solar_Gravity(Switch,t,mu_sun,mu_moon,r_sat,initial_date)

    % Solar Gravity
    %
    % This function and its subfunctions were written by Luis Mendoza
    % 
    % -------------------------------------------------------------------------
    % Inputs
    % -------------------------------------------------------------------------
    %
    %    Switch : Switch to turn accelerations on or off
    % 
    %         t : time [s]
    % 
    %    mu_sun : Standard gravitational parameter of the Sun
    % 
    %   mu_moon : standard gravitational parameter of the Moon
    % 
    %     r_sat : ECIF position of spacecraft
    % 
    
    r_sun = Sun_Position(t,initial_date); % Sun's Geocentric Position (ECI)
    
    r_sun_sat = r_sun - r_sat; % Sun's Relative Position wrt Satellite [km] (ECI)
    
    q = dot(r_sat,2*r_sun-r_sat)/norm(r_sun)^2;
    Gamma_Sun = (q^2-3*q+3)/(1+(1-q)^1.5)*q;
    
    a_sun = Switch*mu_sun/norm(r_sun_sat)^3*(Gamma_Sun*r_sun - r_sat); % Acceleration Due to Solar Gravity [km/s^2] (ECI)
    
    % Lunar Gravity
    
    r_moon = Moon_Position(t,initial_date); % Moon's Position Vector (ECI) [km]
    r_moon_sat = r_moon - r_sat; % Moon's Relative Position wrt S/C (ECI) [km]
    
    a_lunar = Switch*mu_moon*(r_moon_sat/(norm(r_moon_sat)^3) - r_moon/(norm(r_moon)^3)); % Acceleration Due to Lunar Gravity [km/s^2] (ECI)

end