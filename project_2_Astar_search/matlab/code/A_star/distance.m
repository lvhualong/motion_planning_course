function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
% Euclidean heuristic
% dist=sqrt((x1-x2)^2 + (y1-y2)^2);

%Manhattan heuristic
% dist=abs(x1-x2)+abs(y1-y2);

%Diagonal heuristic
 dx = abs(x1 - x2)
 dy = abs(y1 - y2)
 dist =  1.0 * (dx + dy) + (sqrt(2) - 2 ) * min(dx, dy);