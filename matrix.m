function [y] = matrix(alpha, a, d, theta)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
y = [cosd(theta), -sind(theta), 0, a;
     sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -d*sind(alpha);
     sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), d*cosd(alpha);
     0, 0, 0, 1];
end

