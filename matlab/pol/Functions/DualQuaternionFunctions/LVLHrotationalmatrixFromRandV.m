function T = LVLHrotationalmatrixFromRandV(r_ECIF,v_ECIF)

    % Unit vectors from "Fundamentals of Spacecraft Attitude Determination and
    % Control" that represent the LVLH Frame
    
    o3 = -r_ECIF/norm(r_ECIF); % Nadar pointing: Corresponds to z axis of GRACE
    
    o2 = -cross(r_ECIF,v_ECIF)/norm(cross(r_ECIF,v_ECIF)); % Nadar pointing: Corresponds to y axis of GRACE
    
    o1 = cross(o2,o3); % Nadar pointing: Corresponds to x axis of GRACE
    
    T = [o1';o2';o3'];

end