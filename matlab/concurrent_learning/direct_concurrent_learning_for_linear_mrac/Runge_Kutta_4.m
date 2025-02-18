function [t, x] = Runge_Kutta_4(h, t0, tf, x0, dynamics)   
    t = t0:h:tf;                                         % Calculates upto y(3)
    % x = zeros(1,length(t)); 
    x = x0;

    for i=1:(length(t)-1)                              % calculation loop
        k_1 = dynamics(t(i),x(:, i));
        k_2 = dynamics(t(i)+0.5*h,x(:,i)+0.5*h*k_1);
        k_3 = dynamics((t(i)+0.5*h),(x(:,i)+0.5*h*k_2));
        k_4 = dynamics((t(i)+h),(x(:,i)+k_3*h));
    
        x(:,i+1) = x(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;  % main equation
    end
    x = x';
    t = t';
end