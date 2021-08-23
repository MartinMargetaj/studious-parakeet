classdef plotter < handle
    %PLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vehicle
        vehicleBorder
        wheelSize = 0.2;
        FLWheelAll
        FRWheelAll
        RLWheelAll
        RRWheelAll
        maxTurnRadius = 30;
        turnRadiusPoint
    end
    
    methods
        function obj = plotter(vehicle_)
            obj.vehicle = vehicle_;
        end
        
        function obj = update(obj)
            vehicle_ = obj.vehicle;
            rFront = sqrt((vehicle_.height*vehicle_.ratioT)^2+(vehicle_.width/2)^2);
            rRear = sqrt((vehicle_.height*(1-vehicle_.ratioT))^2+(vehicle_.width/2)^2);
            FLangle = atan((vehicle_.width/2)/(vehicle_.height*(vehicle_.ratioT)));
            RLangle = pi - atan((vehicle_.width/2)/(vehicle_.height*(1-vehicle_.ratioT)));
            RRangle = -RLangle;
            FRangle = -FLangle;
            FL = [vehicle_.x + rFront * cos(FLangle+vehicle_.psi); vehicle_.y + rFront * sin(FLangle+vehicle_.psi)];
            FR = [vehicle_.x + rFront * cos(FRangle+vehicle_.psi); vehicle_.y + rFront * sin(FRangle+vehicle_.psi)];             
            RL = [vehicle_.x + rRear  * cos(RLangle+vehicle_.psi); vehicle_.y + rRear  * sin(RLangle+vehicle_.psi)];
            RR = [vehicle_.x + rRear  * cos(RRangle+vehicle_.psi); vehicle_.y + rRear  * sin(RRangle+vehicle_.psi)];
            obj.vehicleBorder = [FL,FR,RR,RL,FL];
            rFront = sqrt((vehicle_.wheelBaseX*(vehicle_.ratioT))^2+(vehicle_.wheelBaseY/2)^2);
            rRear = sqrt((vehicle_.wheelBaseX*(1-vehicle_.ratioT))^2+(vehicle_.wheelBaseY/2)^2);
            FLangle = atan((vehicle_.wheelBaseY/2)/(vehicle_.wheelBaseX*(vehicle_.ratioT)));
            RLangle = pi - atan((vehicle_.wheelBaseY/2)/(vehicle_.wheelBaseX*(1-vehicle_.ratioT)));
            RRangle = -RLangle;
            FRangle = -FLangle;
            FLWheel = [vehicle_.x + rFront * cos(FLangle+vehicle_.psi); vehicle_.y + rFront * sin(FLangle+vehicle_.psi)];
            FRWheel = [vehicle_.x + rFront * cos(FRangle+vehicle_.psi); vehicle_.y + rFront * sin(FRangle+vehicle_.psi)];             
            RLWheel = [vehicle_.x + rRear  * cos(RLangle+vehicle_.psi); vehicle_.y + rRear  * sin(RLangle+vehicle_.psi)];
            RRWheel = [vehicle_.x + rRear  * cos(RRangle+vehicle_.psi); vehicle_.y + rRear  * sin(RRangle+vehicle_.psi)];
            FLWheelF = [FLWheel(1)+ obj.wheelSize * cos(vehicle_.deltaF+vehicle_.psi); FLWheel(2) + obj.wheelSize * sin(vehicle_.deltaF+vehicle_.psi)];
            FLWheelR = [FLWheel(1)+ obj.wheelSize * cos(-pi+vehicle_.deltaF+vehicle_.psi); FLWheel(2) + obj.wheelSize * sin(-pi+vehicle_.deltaF+vehicle_.psi)];
            FRWheelF = [FRWheel(1)+ obj.wheelSize * cos(vehicle_.deltaF+vehicle_.psi); FRWheel(2) + obj.wheelSize * sin(vehicle_.deltaF+vehicle_.psi)];
            FRWheelR = [FRWheel(1)+ obj.wheelSize * cos(-pi+vehicle_.deltaF+vehicle_.psi); FRWheel(2) + obj.wheelSize * sin(-pi+vehicle_.deltaF+vehicle_.psi)];
            RLWheelF = [RLWheel(1)+ obj.wheelSize * cos(vehicle_.deltaR+vehicle_.psi); RLWheel(2) + obj.wheelSize * sin(vehicle_.deltaR+vehicle_.psi)];
            RLWheelR = [RLWheel(1)+ obj.wheelSize * cos(-pi+vehicle_.deltaR+vehicle_.psi); RLWheel(2) + obj.wheelSize * sin(-pi+vehicle_.deltaR+vehicle_.psi)];
            RRWheelF = [RRWheel(1)+ obj.wheelSize * cos(vehicle_.deltaR+vehicle_.psi); RRWheel(2) + obj.wheelSize * sin(vehicle_.deltaR+vehicle_.psi)];
            RRWheelR = [RRWheel(1)+ obj.wheelSize * cos(-pi+vehicle_.deltaR+vehicle_.psi); RRWheel(2) + obj.wheelSize * sin(-pi+vehicle_.deltaR+vehicle_.psi)];
            obj.FLWheelAll = [FLWheelF,FLWheelR];
            obj.FRWheelAll = [FRWheelF,FRWheelR];
            obj.RLWheelAll = [RLWheelF,RLWheelR];
            obj.RRWheelAll = [RRWheelF,RRWheelR];
            if (vehicle_.deltaF ~= 0) || (vehicle_.deltaR ~= 0)
                rFront = (vehicle_.wheelBaseX*(vehicle_.ratioT));
                rRear = (vehicle_.wheelBaseX*(1-vehicle_.ratioT));
                frontTurnLineStart = [vehicle_.x + rFront * cos(vehicle_.psi); vehicle_.y + rFront * sin(vehicle_.psi)];
                rearTurnLineStart = [vehicle_.x - rRear * cos(vehicle_.psi); vehicle_.y - rRear * sin(vehicle_.psi)];
                frontTurnLineStartLeft = [frontTurnLineStart(1) + obj.maxTurnRadius * cos(vehicle_.psi+vehicle_.deltaF+pi/2); frontTurnLineStart(2) + obj.maxTurnRadius * sin(vehicle_.psi+vehicle_.deltaF+pi/2)];
                frontTurnLineStartRight = [frontTurnLineStart(1) + obj.maxTurnRadius * cos(vehicle_.psi+vehicle_.deltaF-pi/2); frontTurnLineStart(2) + obj.maxTurnRadius * sin(vehicle_.psi+vehicle_.deltaF-pi/2)];
                rearTurnLineStartLeft = [rearTurnLineStart(1) + obj.maxTurnRadius * cos(vehicle_.psi+vehicle_.deltaR+pi/2); rearTurnLineStart(2) + obj.maxTurnRadius * sin(vehicle_.psi+vehicle_.deltaR+pi/2)];
                rearTurnLineStartRight = [rearTurnLineStart(1) + obj.maxTurnRadius * cos(vehicle_.psi+vehicle_.deltaR-pi/2); rearTurnLineStart(2) + obj.maxTurnRadius * sin(vehicle_.psi+vehicle_.deltaR-pi/2)];
                [x,y] = intersections([frontTurnLineStartLeft(1),frontTurnLineStartRight(1)],[frontTurnLineStartLeft(2),frontTurnLineStartRight(2)],[rearTurnLineStartLeft(1),rearTurnLineStartRight(1)],[rearTurnLineStartLeft(2),rearTurnLineStartRight(2)]);
                if ~isempty(x)
                    obj.turnRadiusPoint = [frontTurnLineStart(1),x,rearTurnLineStart(1);frontTurnLineStart(2),y,rearTurnLineStart(2)];
                else
                    obj.turnRadiusPoint = [];
                end
            else
                obj.turnRadiusPoint = [];
            end
        end
        
        function obj = plot_pos(obj)
                clf
                plot(obj.vehicleBorder(1,:),obj.vehicleBorder(2,:),'k')
                hold on
                plot(obj.vehicle.x,obj.vehicle.y,'kx')
                plot(obj.FLWheelAll(1,:),obj.FLWheelAll(2,:),'r','LineWidth', 2)
                plot(obj.FRWheelAll(1,:),obj.FRWheelAll(2,:),'r','LineWidth', 2)
                plot(obj.RLWheelAll(1,:),obj.RLWheelAll(2,:),'r','LineWidth', 2)
                plot(obj.RRWheelAll(1,:),obj.RRWheelAll(2,:),'r','LineWidth', 2)
                obj.plot_env;
                if ~isempty(obj.turnRadiusPoint)
                    plot(obj.turnRadiusPoint(1,:),obj.turnRadiusPoint(2,:),'r','LineWidth', 1.2)
                end
                obj.plot_sensors;
                prediction = obj.vehicle.sim_fure_steps;
                plot(prediction(1,:),prediction(2,:),'r--')
                pause(0.1)
        end
        
        function obj = plot_env(obj)
            for i = 1:numel(obj.vehicle.enviroment.lines)
               hold on  
               plot(obj.vehicle.enviroment.lines{i}(1,:),obj.vehicle.enviroment.lines{i}(2,:),'b','LineWidth', 1.2)
            end
            xlim(obj.vehicle.enviroment.xLim)
            ylim(obj.vehicle.enviroment.yLim)
        end
        
        function obj = plot_sensors(obj)
            for i = 1:numel(obj.vehicle.sensors)
                actSnrs = obj.vehicle.sensors{i};
                for j = 1:size(actSnrs.lasers,2)
                    plot([actSnrs.actSnsrPos(1),actSnrs.lasers{1,j}(1)],[actSnrs.actSnsrPos(2),actSnrs.lasers{1,j}(2)],actSnrs.color)
                end
            end
        end
    end
    
end

