function place_obstacles(Obs)
[nx,ny]=size(Obs)
for i=1:nx
    obstacle(i,:)=Obs(i,:)
    plot(obstacle(i,1),obstacle(i,2),'r*')
    hold on
end
    