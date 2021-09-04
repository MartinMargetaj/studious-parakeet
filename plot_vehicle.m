function plot_vehicle(x,y,psi,width,height)
ratioT = 0.5;
rFront = sqrt((height*ratioT)^2+(width/2)^2);
rRear = sqrt((height*(1-ratioT))^2+(width/2)^2);
FLangle = atan((width/2)/(height*(ratioT)));
RLangle = pi - atan((width/2)/(height*(1-ratioT)));
RRangle = -RLangle;
FRangle = -FLangle;
FL = [x + rFront * cos(FLangle+psi); y + rFront * sin(FLangle+psi)];
FR = [x + rFront * cos(FRangle+psi); y + rFront * sin(FRangle+psi)];
RL = [x + rRear  * cos(RLangle+psi); y + rRear  * sin(RLangle+psi)];
RR = [x + rRear  * cos(RRangle+psi); y + rRear  * sin(RRangle+psi)];
obj.vehicleBorder = [FL,FR,RR,RL,FL];
plot(obj.vehicleBorder(1,2:5),obj.vehicleBorder(2,2:5),'k')
plot(obj.vehicleBorder(1,1:2),obj.vehicleBorder(2,1:2),'r')



end