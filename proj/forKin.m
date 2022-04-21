function [Oi0,zi0,n,T] = forKin(theta,den)


% theta=[10 40 -20]*(pi/180);  %initial
%    theta=[140 60 -90];    %final

 theta=theta;% conversion from degree to radian
% fi =0;      


alpha=[0 0 0];
ai=[0  1  1];
di=[0 0 0];
%   den=[0.5 0 0]';
 
n=3;
% den=[cos(fi);-sin(fi);0];


Temp = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for i=1:n
    T_i_i_1= [  cosd(theta(i))                           -sind(theta(i))                         0                          ai(i)
                sind(theta(i))*cosd(alpha(i))             cosd(theta(i))*cos(alpha(i))         -sind(alpha(i))        -sin(alpha(i))*di(i)
                sind(theta(i))*sind(alpha(i))             cosd(theta(i))*sin(alpha(i))          cosd(alpha(i))         cos(alpha(i))*di(i)
                0                                           0                                    0                           1                  ];      
    
    Ti0(:,:,i)=Temp*T_i_i_1;
    Temp=Ti0(:,:,i);
    
    Oi0(1:3,i)=Ti0(1:3,4,i);
    zi0(1:3,i)=Ti0(1:3,3,i);
   
     
    
end
T= Temp(1:3,1:3);
Pe0=Oi0(:,n) + Ti0(1:3,1:3,n)*(den);
Oi0(:,(n+1))=Pe0;
Oi0(1:3,n+2)=(Oi0(1:3,1)+Oi0(1:3,2))/2;
Oi0(1:3,n+3)=(Oi0(1:3,2)+Oi0(1:3,3))/2;
Oi0(1:3,n+4)=(Oi0(1:3,3)+Oi0(1:3,4))/2;

end