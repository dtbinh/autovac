%behaviour go-to-goal
%simulates the go-to-goal behaviour
function [uGTG]=GTG(xgoal,xnew,dt)
%kp=1;
%error= sqrt(xgoal(2,1)-xnew(2)^2+xgoal(1,1)-xnew(1)^2);
v_cal1=1;% 10*error*dt;
phi_des=atan2(xgoal(2,1)-xnew(2),xgoal(1,1)-xnew(1))*dt;
phi_act=xnew(3);
error=phi_des-phi_act;
phi_cal1=error;
uGTG=[v_cal1;phi_cal1];
    