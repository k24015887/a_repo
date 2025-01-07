function [x,y] = forward_kinematics(r1, r2, t1, t2)
% to return the desired position(x,y) for ploting points onto workspace
% input:
% r1 is reararm lenth[mm]
% r2 is forearm lenth[mm]
% t1 is desired reararm angle[deg]
% t2 is desired forearm angle[deg]

% the first homogeneous transformation matrix
A01 = [cosd(t1) -sind(t1) 0 r1*cosd(t1);
   sind(t1) cosd(t1) 0 r1*sind(t1);
   0 0 1 0; 
   0 0 0 1];

% the second homogeneous transformation matrix
A12 = [cosd(t2) -sind(t2) 0 r2*cosd(t2);
   sind(t2) cosd(t2) 0 r2*sind(t2);
   0 0 1 0; 
   0 0 0 1];

% calculate T
T = A01*A12;

% get value of x and y from matrix T
x = T(1,4); 
y = T(2,4); 

% or
%T = [cosd(t1+t2) -sind(t1+t2) 0 r1*cosd(t1)+r2*cosd(t1+t2);
%     sind(t1+t2) sind(t1+t2) 1 r1*sind(t1)+r2*sind(t1+t2);
%     0 0 1 0; 
%     0 0 0 1];

end

