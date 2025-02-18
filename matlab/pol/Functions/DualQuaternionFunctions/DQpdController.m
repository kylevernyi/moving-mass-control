function [f_hat] = DQpdController(A,B,A_desired,B_desired,K_A,K_B)
% Dual Quaternion PD-type Controller
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

f_hat = DQmatrixmult(K_A,A_desired - A) + DQmatrixmult(K_B,B_desired - B);

end