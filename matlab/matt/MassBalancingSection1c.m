%Mass-Balancing Simulation Code
%Assuming J is known and constant

%General Notes:
% - omega is represented as 'w' in this code
% - coordinate system is that of conventional airplanes (positive x-axis out
%   the nose, positive y-axis out the right wing, posit
% - J is from Pol and represent the actual testbed.
% - Impliments Concurrent Learning to further estimate r_off

%% Standard Keeping %%
clear all;
close all;
clc;

%% Control Inputs %%
CL_on = 1; %Set to 1 for Concurrent Learning on, Set to 0 for Concurrent Learning off


wd = [0.25;.25;-.25]; %Desired angular velocity in the form [w_x, w_y, w_z]
gamma = 1; %Control Parameter. Should be > 0
gamma_ad = .01; %Learning Rate for adaptive controller specifically
gamma_CL = .0000001*diag([1, 0.01, 10000]); %Learning Rate for Concurrent Learning elements specifically
pbar = 15; %Number of recorded states for Concurrent Learning

E = [0 0 0]; %Initial vehicle position in Euler Angles (Roll, Pitch, Yaw) Using standard airplane coordinate system

Theta_hat = [0; 0; 0]; %Intial Guess of Roff Vector
w = [0; 0; 0]; %Initial Angular Velocity

%% Vehicle Parameters %%
J = diag([0.0226 0.0226 0.0226]); %Vehicle Inertia Tensor
Theta = [.05; .05; 0]; %Roff Vector from Center of Rotation to Center of Mass
m = 4.2; %Vehicle Mass (kg)

%% Sim Parameters %%
Tf = 20; %Final Time (s)
dt = 0.001; %Timeset Size (s)
g = [0;0;9.8]; %Gravity Vector 

%% Initial Matrices/Calculations %%
%Matrices for Recording Data
q_rec = [];
w_til_rec = [];
w_rec = [];
wd_rec = [];
Theta_til_rec = [];
Theta_hat_rec = [];
Theta_rec = [];
Eul_rec = [];
Phi_rec = [];
sigma_rec = [];

%Initial Variables
p = 0; %Number of recorded states
tau = [0; 0; 0]; %Set for initial step
Phi_last = zeros(3,3);

Deltat = zeros(1,3*pbar); %Defining a 3x1xpbar tensor
Zt = zeros(3,3*pbar); %Defining a 3x3xpbar tensor
Epsilont = zeros(1,3*pbar); %Defining a 3x1xpbar tensor

%Calculations
q = transpose(compact(quaternion(E,"eulerd","ZYX","point"))); %Converting Euler Angle into Quaternions


%% Simulation %%
for t = 0:dt:Tf
    %Calculate Simulation Parameters
    Theta_til = Theta_hat - Theta;     %Calculating Theta Tilde 
    w_til = w - wd;                    %Calculating Omega(w) Tilde

    Phi = CalcPhi(g,q,m); %Calculating Phi in Body Coordinate
    w_dot = inv(J)*(cross(J*w,w)+Phi*Theta+tau);%Calculating w_dot

    T_Phi = transpose(Phi);
    
    %Updating Concurrent Learning 
    if(p==0)
        p = p+1;
        Zt(:,p:p+2)= T_Phi;
        
        Deltat(:,p:p+2) = (J*w_dot - cross(J*w,w)-tau)';
        Epsilont = Deltat - Theta_hat'*Zt;
    elseif ((norm(Phi-Phi_last)/norm(Phi)) >= 0.081 || rank([Zt T_Phi])>rank(Zt)) % something in the proof 0.081 ?
        if(p<pbar)
             p = p+1;
             i = 3*p-2;
             Zt(:,i:i+2) = T_Phi;

             Deltat(:,i:i+2) = (J*w_dot - cross(J*w,w)-tau)';
             Epsilont = Deltat - Theta_hat'*Zt;
        else
            T = Zt;
            SVold  = min(svd(T));
            for j=1:p
                        ii = 3*j - 2;
                        Zt(:,ii:ii+2) = T_Phi; 

                        SV(j) = min(svd(Zt));    
                        Zt = T;
            end
            [maxSV,Index]=max(SV);
            if(maxSV>SVold)
                iii = 3*Index-2;
                Zt(:,iii:iii+2)= T_Phi;

                Deltat(:,Index:Index+2)= (J*w_dot - cross(J*w,w)-tau)';
                Epsilont = Deltat - Theta_hat'*Zt;
            end      
        end 
    end
    

    %Update Laws
    Adapt_Law_CL = zeros(3,1);
    for k = 1:p
        Cur_Epsilont = Epsilont(:,k:k+2);
        Adapt_Law_CL = Adapt_Law_CL + Zt(:,k:k+2)*Cur_Epsilont';
    end


    Theta_hat_dot = gamma_ad*transpose(Phi)*w_til + CL_on*gamma_CL*Adapt_Law_CL;
    Theta_hat = Theta_hat + dt*(Theta_hat_dot); %Theta Hat (Estimate of r_off)

    %Dynamics
    tau = -Phi*Theta_hat - cross(J*w,wd)-gamma*w_til; %Calculate Reference Input Torque
    w = w + dt*(inv(J)*(cross(J*w,w)+Phi*Theta+tau)); %Update w

    %Kinematics
    q = q + dt*(.5*CalcG(w)*q); 
    
    %Updates
    Phi_last = Phi;

    %Recording
    q_rec = [q_rec q];
    w_til_rec = [w_til_rec w_til]; 
    w_rec = [w_rec w];
    wd_rec = [wd_rec wd];
    Theta_til_rec = [Theta_til_rec Theta_til]; 
    Theta_hat_rec = [Theta_hat_rec Theta_hat]; 
    Theta_rec = [Theta_rec Theta];
    sigma_rec = [sigma_rec min(svd(Zt))];
end

t_rec = 0:dt:Tf;

%% Post Processing %%
for i = 1:length(q_rec) %Generating Euler Angles based on Quaternions
    Eul_rec(:,i) = (180/pi())*transpose(quat2eul(transpose(q_rec(:,i)),'ZYX'));
end

%% Plotting %%
font_size = 22; %Font size on Plots

%%%%%%%%%%% Orientation %%%%%%%%%%%

figure() %Quaternion Angle Plots
subplot(4, 1, 1)
plot(t_rec,q_rec(1,:))
title('Identity Element','FontSize',font_size)

subplot(4, 1, 2)
plot(t_rec,q_rec(2,:))
title('i','FontSize',font_size)

subplot(4, 1, 3)
plot(t_rec,q_rec(3,:))
title('j','FontSize',font_size)

subplot(4, 1, 4)
plot(t_rec,q_rec(4,:))
title('k','FontSize',font_size)
sgtitle('Quaternions','FontSize',font_size)

% 
% figure() %Euler Angle Plots
% subplot(3, 1, 1)
% plot(t_rec,Eul_rec(1,:))
% title('X (Roll)','FontSize',font_size)
% ylabel('Degrees','FontSize',font_size)
% 
% subplot(3, 1, 2)
% plot(t_rec,Eul_rec(2,:))
% title('Y (Pitch)','FontSize',font_size)
% ylabel('Degrees','FontSize',font_size)
% 
% subplot(3, 1, 3)
% plot(t_rec,Eul_rec(3,:))
% title('Z (Yaw)','FontSize',font_size)
% ylabel('Degrees','FontSize',font_size)
% sgtitle('Euler Angles','FontSize',font_size)

%%%%%%%%%%% Theta %%%%%%%%%%%

figure() %Theta Tilde Plot
subplot(3, 1, 1)
plot(t_rec,Theta_til_rec(1,:))
yline(0)
ylabel('m','FontSize',font_size)
title('$\tilde{\Theta}_1$','Interpreter','latex','FontSize',font_size)

subplot(3, 1, 2)
plot(t_rec,Theta_til_rec(2,:))
yline(0)
ylabel('m','FontSize',font_size)
title('$\tilde{\Theta}_2$','Interpreter','latex','FontSize',font_size)

subplot(3, 1, 3)
plot(t_rec,Theta_til_rec(3,:))
yline(0)
ylabel('m','FontSize',font_size)
xlabel('time (s)','FontSize',font_size)
title('$\tilde{\Theta}_3$','Interpreter','latex','FontSize',font_size)
sgtitle('$\tilde{\Theta}$','Interpreter','latex','FontSize',font_size)


% figure() %Theta_hat vs Theta Plots
% subplot(3, 1, 1)
% hold on
% plot(t_rec,Theta_rec(1,:),"--b")
% plot(t_rec,Theta_hat_rec(1,:),"-",'Color',[1 0.5 0])
% legend('$\Theta$','$\hat{\Theta}$','Interpreter','latex','FontSize',font_size)
% hold off
% title('$\Theta_1$','Interpreter','latex','FontSize',font_size)
% 
% subplot(3, 1, 2)
% hold on
% plot(t_rec,Theta_rec(2,:),"--b")
% plot(t_rec,Theta_hat_rec(2,:),"-",'Color',[1 0.5 0])
% hold off
% title('$\Theta_2$','Interpreter','latex','FontSize',font_size)
% 
% subplot(3, 1, 3)
% hold on
% plot(t_rec,Theta_rec(3,:),"--b")
% plot(t_rec,Theta_hat_rec(3,:),"-",'Color',[1 0.5 0])
% hold off
% title('$\Theta_3$','Interpreter','latex','FontSize',font_size)
% sgtitle('$\hat{\Theta}$ vs $\Theta$','Interpreter','latex','FontSize',font_size)

%%%%%%%%%%% Omega %%%%%%%%%%%

figure() %Omega(w) Tilde Plots
subplot(3, 1, 1)
plot(t_rec,w_til_rec(1,:))
yline(0)
ylabel('rad/s','FontSize',font_size)
title('$\tilde{\omega}_1$','Interpreter','latex','FontSize',font_size)

subplot(3, 1, 2)
plot(t_rec,w_til_rec(2,:))
yline(0)
ylabel('rad/s','FontSize',font_size)
title('$\tilde{\omega}_2$','Interpreter','latex','FontSize',font_size)

subplot(3, 1, 3)
plot(t_rec,w_til_rec(3,:))
yline(0)
ylabel('rad/s','FontSize',font_size)
xlabel('time (s)','FontSize',font_size)
title('$\tilde{\omega}_3$','Interpreter','latex','FontSize',font_size)
sgtitle('$\tilde{\omega}$','Interpreter','latex','FontSize',font_size)


% figure() %Omega Desired (wd) vs Omega(w) Plots
% subplot(3, 1, 1)
% hold on
% plot(t_rec,wd_rec(1,:),'--b')
% plot(t_rec,w_rec(1,:),"-",'Color',[1 0.5 0])
% legend('$\omega_d$','$\omega$','Interpreter','latex','FontSize',font_size)
% hold off
% title('$\omega_1$','Interpreter','latex','FontSize',font_size)
% 
% subplot(3, 1, 2)
% hold on
% plot(t_rec,wd_rec(2,:),'--b')
% plot(t_rec,w_rec(2,:),"-",'Color',[1 0.5 0])
% hold off
% title('$\omega_2$','Interpreter','latex','FontSize',font_size)
% 
% subplot(3, 1, 3)
% hold on
% plot(t_rec,wd_rec(3,:),'--b')
% plot(t_rec,w_rec(3,:),"-",'Color',[1 0.5 0])
% hold off
% title('$\omega_3$','Interpreter','latex','FontSize',font_size)
% sgtitle('$\omega_d$ vs $\omega$','Interpreter','latex','FontSize',font_size)

%%%%%%%%%%% Sigma %%%%%%%%%%%
if CL_on == 1
    figure() %Sigma through iterations
    plot(t_rec,sigma_rec)
    title('Minimum $\sigma$','Interpreter','latex','FontSize',font_size)
end

%% Functions %%

function G = CalcG(w) %Skew-Symmetric Matrix for quaternion dynamics
    G = [0 w(3) -w(2) w(1);
        -w(3) 0 w(1) w(2);
         w(2) -w(1) 0 w(3);
        -w(1) -w(2) -w(3) 0];
end

function Phi = CalcPhi(g,q,m) %Calculates G by converting G into a skew symmetric matrix times mass
    C = CalcC(q);
    g_body = C*g;
    g_skew = [0 -g_body(3) g_body(2);
              g_body(3) 0 -g_body(1);
              -g_body(2) g_body(1) 0];
    Phi = -m*g_skew;
end

function C = CalcC(q) %Function that converts quaternions into direction cosine matrix
    C = [q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));
         2*(q(2)*q(3)-q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)+q(1)*q(2));
         2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2];
end
