function v_wind_vec_ECIF = UpperLevelWinds(initial_date,time,r_vec)

% Finding the Upper Level Winds----------------------------------------

    utc = initial_date + seconds(time); % Finding the current time of the simulation

    [r_ecef] = eci2ecef(utc,r_vec); % Position of spacecraft in ECEF

    lla = ecef2lla(r_ecef'); % ECEF positon to Latitude, Longitude and Altitude


    % Since the MATLAB Function atmoshwm only works up to 500 KM
    if lla(:,3) > 500000
        lla(:,3) = 500000;
    else
        lla(:,3) = lla(:,3);
    end


    v_wind_NED = atmoshwm(lla(:,1),lla(:,2),lla(:,3)); % Upper level wind 

    v_wind_vec_NED = [v_wind_NED';0];

    Q_NED_ECEF = dcmecef2ned(lla(:,1),lla(:,2)); % From NED to ECEF

    v_wind_vec_ECEF = transpose(Q_NED_ECEF)*v_wind_vec_NED;

    Q_ECEF_ECIF = dcmeci2ecef('IAU-2000/2006',utc);

    v_wind_vec_ECIF = transpose(Q_ECEF_ECIF)*v_wind_vec_ECEF;


    %Check = norm(v_wind_vec_NED) - norm(v_wind_vec_ECEF) + norm(v_wind_vec_NED) - norm(v_wind_vec_ECIF);

end