function [M,z]=Newdist(xobs,xnew)

Dist=[];
Theta=[];

dist=norm(xobs-xnew);
theta=-atan2((xobs(2,1)-xnew(2,1)),(xobs(1,1)-xnew(1,1)));
 
% %    dist=sqrt((x_leader-x_becon)^2+(y_leader-y_becon)^2);
% %    Dist=[Dist dist]
%     Theta=[Theta theta]
    M=[dist;theta]
    z=min(M(1,1))
end