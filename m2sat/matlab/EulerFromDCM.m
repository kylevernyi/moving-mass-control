function [ypr] = EulerFromDCM(C)
%3-2-1 euler angles (yaw pitch roll) from DCM
ypr = [zeros(3,1,length(C))];
for i=1:length(C)
ypr(:,:,i) = [atan2(C(1,2,i),C(1,1,i));
        -asin(C(1,3,i));
        atan2(C(2,3,i),C(3,3,i))];
end
end