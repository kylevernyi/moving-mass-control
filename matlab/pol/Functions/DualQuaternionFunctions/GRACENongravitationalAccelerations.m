function [a_GRACE_approximated] = GRACENongravitationalAccelerations(t,m_B,Cd,mu_sun,mu_moon,initial_date,r_I_BI_vec,omega_Earth_vec,omega_hat_B_BI,q_hat_BI)

    %--------------------------------------------------------------------------
    % Accelerations from Sun and Moon--------------------------------------
    
    [a_sun,a_lunar] = Solar_Gravity(1,t,mu_sun./(10^9),mu_moon./(10^9),r_I_BI_vec./1000,initial_date);
    
    a_sun = a_sun.*1000;
    a_lunar = a_lunar.*1000;
    
    %----------------------------------------------------------------------
    % Finding the Upper Level Winds----------------------------------------
    
    v_wind_vec_ECIF = UpperLevelWinds(initial_date,t,r_I_BI_vec);
    
    
    
    %----------------------------------------------------------------------
    % Finding the relative velocity----------------------------------------
    
    v_vec_rel_I = omega_hat_B_BI(2:4,2) - cross(omega_Earth_vec,r_I_BI_vec) - v_wind_vec_ECIF; % Relative Velocity of Spacecraft wrt Earth's atmosphere and upper level winds
    
    v_rel_B = Qvectorrotation([0;v_vec_rel_I],q_hat_BI(:,1));
    v_rel_B_vec = v_rel_B(2:4,:);
    
    
    v_rel_B_vec_unit = v_rel_B_vec./norm(v_rel_B_vec); 
    
    %----------------------------------------------------------------------
    
    Ac = GRACECrossSectionalArea(v_rel_B_vec); % Cross Sectional Area of GRACE-FO
    
    rho_hat = [];
    % Data Set two
    rho_hat(1,:) = (sin((0.0005535*t) + 0.8)*(7*(10^(-11)))) - (2.8*(10^(-11)));
    
    
    rho_hat(2,:) = rho_hat(1,:);
    rho_hat(3,:) = rho_hat(1,:);
    
    a_drag_approximated = (-1/2)*diag(rho_hat')*((Cd*Ac)/m_B)*((v_rel_B_vec.^2).*v_rel_B_vec_unit);
    
    
    % Unmodeled Dynamics-----------------------------------------------
    
    % X-axis
    
    y1 = 5.02189e-08; 
    y2 = -2.90501e-08;
    
    a = 0.0005535;
    b = 1;
    c = (y1 - y2)/2;
    d = (y1 + y2)/2;
    
    %         a_unmodeled_hat_i(1,:) = sin((a*time) + b)*c + d;
    a_unmodeled(1,:) = 0;
    
    % Y-axis
    
    y1 = -1.00494e-05; 
    y2 = -1.14837e-05;
    
    a = 0.0005535;
    b = 0.5;
    c = (y1 - y2)/2;
    d = (y1 + y2)/2;
    
    a_unmodeled(2,:) = sin((a*t) + b)*c + d;
    
    % Z-axis
    
    y1 = 5.7795e-07; 
    y2 = -9.55741e-07;
    
    a = 0.0005535;
    b = 2.1;
    c = (y1 - y2)/2;
    d = (y1 + y2)/2;
    
    a_unmodeled(3,:) = sin((a*t) + b)*c + d;
    
    
    
     a_GRACE_approximated = a_lunar + a_sun + a_drag_approximated + a_unmodeled;

end