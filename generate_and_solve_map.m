clc
clear
close all

%% vehicle stats
width = 1.760;%m
height = 4.585;%m
maxR = 5; %m

%% ask for world boundaries
% xDim = abs(input('Give me X dimension\n')); 
xDim = 25;yDim = 25;
% yDim = abs(input('Give me Y dimension\n'));
walls = {};
%% transform boundaries to walls
%               x1,x2; y1,y2
walls{end+1} = [0,xDim;0,0];
walls{end+1} = [xDim,xDim;0,yDim];
walls{end+1} = [xDim,0;yDim,yDim];
walls{end+1} = [0,0;yDim,0];


%% draw walls
for i = 1:numel(walls)
    hold on
    plot(walls{i}(1,:),walls{i}(2,:),'b','LineWidth', 1.2)
end
axis('equal')
xlim([-0.5, xDim+0.5])
ylim([-0.5, yDim+0.5])

%% ask for start position of vehicle
disp('Click on start position')
% [xStart, yStart] = ginput(1);
xStart =3; yStart = 3;

plot(xStart,yStart,'r*')

%% ask for phi of vehicle at start
% [xTemp, yTemp] = ginput(1);
% xTemp = xStart - xTemp;
% yTemp = yStart - yTemp;
% psiStart = atan(yTemp/xTemp);
psiStart = pi/2;
plot_vehicle(xStart,yStart,psiStart,width,height)


%% ask for end position of vehicle
disp('Click on end position')
% [xFinal, yFinal] = ginput(1);
xFinal = 20; yFinal = 20;
plot(xFinal,yFinal,'g*')


%% ask for psi of vehicle at end
% [xTemp, yTemp] = ginput(1);
% xTemp = xFinal - xTemp;
% yTemp = yFinal - yTemp;
% psiFinal = atan2(yTemp/xTemp);
psiFinal = 0;
plot([xFinal,xFinal+cos(psiFinal)],[yFinal,yFinal+sin(psiFinal)],'r')

planner = motion_planner(walls,xFinal, yFinal, psiFinal, xStart, yStart, psiStart);

points = planner.basic_trajectory(); 