function pvec(x,y,r,fi,color,style,linewidth)
%% Plotting vectors in polar coordinate system
%  Author: Matevž Lavtar
%% Setting space

x0= x;                            % x origin of system 
y0= y;                            % y origin of system
alpha = deg2rad(fi);             % angle fi from degree to radian
k1=0.05;                         % factor of lenght of arro
k2 = 0.01;                       % factor of width of arrow
theta1 = 20;                     % angle of arrow relative to vector line in degree
theta = theta1*(pi/180);

R = [cos(alpha) -sin(alpha) 0 x0;...  % rotation around z axis
    sin(alpha) cos(alpha) 0 y0;...
    0 0 0 0;...
    0 0 0 1];
% origin vector
p0 = [x0; y0; 0;1];
% start positon of vector
p1 = [x0+r; y0; 0;1];
% arrow
pA = [(x+r)-k1*(r+x); k2*(r+x); 0;1];
pB = [(x+r)-k1*(r+x); -k2*(r+x); 0;1];

p2 = R*p1;
pA2 = R*pA;
pB2 = R*pB;
xA = pA2(1);
yA = pA2(2);
xB = pB2(1);
yB = pB2(2);
xC = p2(1);
yC = p2(2);
if x ~= 0
    yC=r;
    yA= yC-0.05*r;
    yB= yC-0.05*r;
end
%% Default parameters settings
 if ~exist('color','var')
    color = 'blue';
 end
  if ~exist('style','var')
    style = '-';
  end
  if ~exist('linewidth','var')
    linewidth = 1;
 end
%% Plot vector with arrow
plot([x0;xC],[y0;yC],'Color',color,'LineStyle',style,'LineWidth',linewidth)
hold on
plot([xC;xA],[yC; yA],[xC;xB],[yC; yB],'Color',color,'LineStyle','-','LineWidth',linewidth,'HandleVisibility','off')
daspect([1 1 1])
grid on
end