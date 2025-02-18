function [data_product] = DataLinearInterpolation(data_time,data,t)

% This function is made to find the linear interpolated value between data
% points in a time based function
%
% Inputs
%
% data_time........Time corresponding to data
%
% data.............Data to be interpolated
%
% t................Current time



[~,loc] = mink(abs(data_time - t),2);

point1 = max(loc);
point2 = min(loc);


data_time_values = [data_time(point1);data_time(point2)];
data_values = [data(point1);data(point2)];

data_product = interp1(data_time_values,data_values,t);

end