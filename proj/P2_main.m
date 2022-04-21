
clc
clear all
close all
%  thef = [160;45;90];
pf = [-2.081  0.621  0];
qf = 140;
pi = [1.866  1.366  0];
qi = 0;
[thef,den] = inkin(pf,qf);
[Of,~,n] = forKin(thef,den);
% the = [10;40;-20];
[the,den] = inkin(pi,qi);

alpha =0.5;



while  norm(the - thef)>1
     
      [Oi0,zi0,n] = forKin(the,den);
    
    [Torque,b] = Att_field(Oi0,Of,zi0,n);
    
     thetaNew = the + (alpha*Torque)/norm(Torque);          
 
    
   
     X = Oi0(1,1:4);
     Y = Oi0(2,1:4);
     Z = Oi0(3,1:4);
     plot(X,Y,b(1,:),b(2,:),'o')
     axis([-4 4 -3 3])
     pause(0.05)
    
   the = thetaNew;
  
end