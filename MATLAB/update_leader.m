function xNew=update_leader(x_leader,y_leader,phi,u)
x=[x_leader;y_leader;phi];
B=1;
xnew(1) = x(1) + u(1)*cos(x(3)+u(2));
xnew(2) = x(2) + u(1)*sin(x(3)+u(2));
xnew(3) = x(3) + u(1)/B*sin(u(2));
xNew=xnew(:,:);