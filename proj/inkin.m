function [theta,den] = inkin(p,q)
a1 = 1; a2 = 1; a3=0.5;
%   p=[-1.87674713556772  0.770613776753825  0];
%   q= 110;
px = p(1);
py = p(2);
 

den = [a3 0 0]';

wx = px - a3*cosd(q);
wy = py - a3*sind(q);

D = ( wx^2 + wy^2 - a1^2 - a2^2 )/(2*a1*a2);


    
theta_2L = atan2d(sqrt(1-D^2),D);
theta_2R = atand(-sqrt(1-D^2)/D);

theta_1L = atan2d(wy,wx) - atan2d((a2*sind(theta_2L)),(a1+a2*cosd(theta_2L)));
theta_1R = atand(wy/wx) - atand((a2*sind(theta_2R))/(a1+a2*cosd(theta_2R)));

theta_3L =  q - (theta_1L + theta_2L);
theta_3R =  q - (theta_1R + theta_2R);

theta = [theta_1L; theta_2L; theta_3L];


end