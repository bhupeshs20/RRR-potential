
function [Torque,b] = Att_field(Oi0,Of,zi0,n)



 % For Jacobian for origion 2 and 3 and mid points
 J(:,:,1) = [[0 0 0]'         [ 0 0 0]'                             [ 0 0 0]' ];
 J(:,:,2) = [cross(zi0(:,1),(Oi0(:,2)-Oi0(:,1)))         [ 0 0 0]'                             [ 0 0 0]' ];
 J(:,:,3) = [cross(zi0(:,1),(Oi0(:,3)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,3)-Oi0(:,2)))   [ 0 0 0]'];
 J(:,:,4) = [cross(zi0(:,1),(Oi0(:,4)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,4)-Oi0(:,2)))   cross(zi0(:,3),(Oi0(:,4)-Oi0(:,3)))];
 J(:,:,5) = [cross(zi0(:,1),(Oi0(:,5)-Oi0(:,1)))          [0 0 0]'                              [0 0 0]' ];
 J(:,:,6) = [cross(zi0(:,1),(Oi0(:,6)-Oi0(:,1)))          cross(zi0(:,2),(Oi0(:,6)-Oi0(:,2)))   [0 0 0]'];
 J(:,:,7) = [cross(zi0(:,1),(Oi0(:,7)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,7)-Oi0(:,2)))   cross(zi0(:,3),(Oi0(:,7)-Oi0(:,3)))];   

for i=1:(n+4)
    
    % Attractive field
    zita = [1;2;3;1;1;1;1];
    f_att(:,i) = -zita(i)*(Oi0(:,i) - Of(:,i));
    
    
    
    % Repulsive field
    
    
    % constraint motion below horizontal level
    ro = 0.1; % radius of influence
    x = [1;0;0]; 
    eta = 1;
    dist_vec = Oi0(:,i)-x;
    
    if norm(dist_vec)<=ro
        f_repC(:,i) = eta*(1/norm(dist_vec) - 1/ro)*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
        
    elseif norm(dist_vec)>ro
        f_repC(:,i) =[0;0;0];
        
    end
    
    
    
    % point obstacles
    for m=1:3
    ro = 0.3; 
    b = [1    0   -1;
        1.5  1.3   .9;
        0     0    0]; 
    eta = 1;
    dist_vec = Oi0(:,i)-b(:,m);
    
    if norm(dist_vec)<=ro
        f_rep(:,i,m) = eta*(1/norm(dist_vec) - 1/ro)*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
        
    elseif norm(dist_vec)>ro
        f_rep(:,i,m) =[0;0;0];
        
    end
    end
end
    % avoiding self collision
    for j=3:n+4
    ro=0.25;
    dist_vec =Oi0(:,j)-Oi0(:,j-2);
     if norm(dist_vec)<=ro
        f_reps(:,j) = eta*(1/norm(dist_vec) - 1/ro)*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
        
    elseif norm(dist_vec)>ro
        f_reps(:,j) =[0;0;0];
        
    end
    
    end
    
    temp=zeros(3,1);
for k=1:n+4    
for m=1:3
    % torque by attractive
t(:,k)  = J(:,:,k)'*f_att(:,k);

% torque by repulsive field
tr(:,k,m)  = J(:,:,k)'*f_rep(:,k,m);

t_repC(:,k) = J(:,:,k)'*f_repC(:,k);

temp=temp + t(:,k) + tr(:,k,m) + t_repC(:,k);
end
end

tor = temp;

% torque by self collision
 temps=zeros(3,1);
 for l=3:n+4
     
     t_reps(:,l) = J(:,:,l)'*f_reps(:,l);
     temps = temps + t_reps(:,l);
 end
 
% combined torque
Torque  = tor + temps ;





 end


