function [u,error]=select_blend(delta_i,dt,xgoal,xnew,x_obs,iter,Error_sum)
n_d=size(delta_i,1)
alpha=1;
theta=90;
%kpg=1/norm(xgoal-xnew(1:2)')%*exp^N_sim;
[uGTG]=GTG(xgoal,xnew,dt);
%PI controller for go-to-goal
% phi_desired=atan2(xgoal(2,1)-xnew(2),xgoal(1,1)-xnew(1))%*exp^(-N_sim);
% phi_actual=xnew(3);
% error=phi_desired-phi_actual;
% kp=0.8;
% 
% Error_sum=Error_sum+error;
% Error_ave=1/iter*(Error_sum)
% KI=10;
% 
% %PI controller
% PI=kp*error+KI*Error_ave;


%P controller for avoid obstacle
error2=atan2(x_obs(2,1)-xnew(2),x_obs(1,1)-xnew(1));
kp2=1;
P_oBs=kp2*error2;

%Rotation matrices for the cw and ccw controllers
R=[cosd(theta) -sind(theta);sind(theta) cosd(theta)];
R1=[cosd(-theta) -sind(-theta);sind(-theta) cosd(-theta)];
%kp=1/min_dist;
Kp3=1;

for i=1:n_d
    sel_con=i;
     if(delta_i(i)>=0.1)
switch sel_con
    case 1
        v=1; 
        %PI=kp*(1/error)+ki;
        phi=PI*dt;
       u=[v;phi]
    case 2
       v=1%abs((min_dist))*dt; 
       phi=-kp2*atan2(x_obs(2,1)-xnew(2),x_obs(1,1)-xnew(1))+pi*dt;
       u=[v;phi]
    case 3
       v=1%abs((min_dist))*dt; 
       phi=delta_i(3,1)*(atan2(x_obs(2,1)-xnew(2),x_obs(1,1)-xnew(1))+pi/2); %atan2(xgoal(2,1)-xnew(2),xgoal(1,1)-xnew(1))*dt;
       uA0=[v;phi]
       u=alpha*R1*uA0*dt
       %u(1,1)=v;
       %u(2,1)=u(2,1);
       
  case 4
       v=1%abs(min_dist)*dt; 
       phi=-delta_i(4,1)*1/error*(atan2(x_obs(2,1)-xnew(2),x_obs(1,1)-xnew(1))+pi/2); %atan2(xgoal(2,1)-xnew(2),xgoal(1,1)-xnew(1))*dt;
       uA0=[v;phi]
       u=alpha*R*uA0*dt
       u(1,1)=v;
       %u(2,1)=u(1,2);
       
  otherwise
        sprintf('%s','something is perhaps wrong' )
end
    
        
    end
%u=[u1 u2 u3 u4]
end

end