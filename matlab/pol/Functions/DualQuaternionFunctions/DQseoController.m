function [f_hat] = DQseoController(omega_hat_TM_TMB,q_hat_TMB,K_TM_p)
% Dual Quaternion Controller Based on Dr. Seo's Paper
% Inputs: (Proportional Dual Quaternion,Derivative Dual Quaternion,
% Desired Proportional Dual Quaternion,Desired Derivative Dual Quaternion,
% Proportional Gain Matrix,Derivative Gain Matrix)
%
% Dimensions: 
%         Proportional Dual Quaternion:    [4x1,4x1]
%           Derivative Dual Quaternion:    [4x1,4x1]
% Desired Proportional Dual Quaternion:    [4x1,4x1]
%   Desired Derivative Dual Quaternion:    [4x1,4x1]
%             Proportional Gain Matrix:    8x8
%               Derivative Gain Matrix:    8x8

one_hat = [1;0;0;0] + zeros(4,1);

s_hat = omega_hat_TM_TMB + K_TM_p;

end