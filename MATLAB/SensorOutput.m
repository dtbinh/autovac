function [M,z]=SensorOutput(leader_pos,B)
x_leader=leader_pos(:,1)
y_leader=leader_pos(:,2)
Pos_leader=[x_leader;y_leader];
[rb,cb]=size(B);
Dist=[];
Theta=[];
for i=1:rb
    x_becon=B(i,1);
    y_becon=B(i,2);
    X_becon=[x_becon;y_becon];
    dist=sqrt((x_leader-x_becon)^2+(y_leader-y_becon)^2);
    Dist=[Dist dist]
    theta=-atan2((y_becon-y_leader),(x_becon-x_leader));
    Theta=[Theta theta]
    M=[Dist;Theta]
 end
    z=min(M(1,:))
    
end

% function [zhat, dh] = beacon(x,X)
% 
% d = sqrt((x(1)-X(1))^2 + (x(2)-X(2))^2)
% 
% zhat = [ d ; -atan2(X(2)-x(2),X(1)-x(1))-x(3) ]
%       
% dh = [-(x(1)-X(1))/d     -(x(2)-X(2))/d     0
%       (x(2)-X(2))/(d^2) (x(1)-X(1))/(d^2) -1 ];