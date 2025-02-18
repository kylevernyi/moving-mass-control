function r_sun = Sun_Position(t,initial_date)

% DESCRIPTION: Computes the Sun's Position Vector in km;

% Coordinate System: ECI

% Constants

AU  = 149597870.691; % Earth-Sun Distance [km]

% Initial Time: 01 Jan 2022 at 0:00 AM UT

initial_date = datetime(2022,6,23,0,0,0);
final_date = initial_date + seconds(t);

% Year-Month-Day-Hour-Minute-Second Value of Final Date

[year, month, dias] = ymd(final_date);
[hour, minute, sec] = hms(final_date);

Date = [year month dias hour minute sec];

% Julian Date

%JD = Julian_Date(Date);
JD = juliandate(Date);

% Number of Days Since J2000

n = JD - 2451545;

% Mean Anomaly of the Sun [deg]

M = 357.529 + 0.98560023*n;
M = mod(M,360);

% Mean Longitude of the Sun [deg]

L = 280.459 + 0.98564736*n;
L = mod(L,360);

% Apparent Solar Ecliptic Longitude [deg]

lambda = L + 1.915*sind(M) + 0.0200*sind(2*M);
lambda = mod(lambda,360);

% Obliquity [deg]

epsilon = 23.439 - 3.56*(10^-7)*n;

% Unit Vector Direction from Earth to Sun

u_hat = [cosd(lambda); sind(lambda)*cosd(epsilon); sind(lambda)*sind(epsilon)];

% Distance b/t Sun and Earth [km]

r_sun_norm = (1.00014 - 0.0167*cosd(M) - 0.000140*cosd(2*M))*AU;

% Output: Sun's Geocentric Position Vector [km]

r_sun = r_sun_norm*u_hat;

end