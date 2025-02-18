function [r_ddot_I_DI_vec,q_DI,T_DI] = DFDynamicsKepler(r_dot_I_DI_vec,r_I_DI_vec,mu)

global q_DI_prev

r_ddot_I_DI_vec = (-mu/(norm(r_I_DI_vec)^3))*r_I_DI_vec;


[q_DI,~] = DCM2quat_LVLH(r_I_DI_vec,r_dot_I_DI_vec,q_DI_prev);


T_DI = LVLHrotationalmatrixFromRandV(r_I_DI_vec,r_dot_I_DI_vec);

q_DI_prev = q_DI;

end