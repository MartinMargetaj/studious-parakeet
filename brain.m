classdef brain < handle
    %BRAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensors = {};
        vehicle
        obstacles
        knownEnviroment
        predictionStep = 5;
        width
        height
        ratioT
        x
        y
        xDot
        yDot
        v
        deltaF
        deltaR
        psiDot
        psi
        beta
        wheelBaseX
        simStep
        xTarget = 8
        yTarget = 8
    end
    
    methods
        function obj = brain(sensors_,vehicle_)
            obj.sensors = sensors_;
            obj.vehicle = vehicle_;
            obj.width = vehicle_.width;
            obj.height = vehicle_.height;
            obj.ratioT = vehicle_.ratioT;
            obj.wheelBaseX = vehicle_.wheelBaseX;
            obj.simStep = vehicle_.simStep*25;
        end
        
        function [dF,dR,V] = ask_what_next(obj, x, y, xDot, yDot, v, deltaF, deltaR, psiDot, psi, beta)
            obj.knownEnviroment = obj.get_known_space();
            obj.x = x;
            obj.y = y;
            obj.xDot = xDot;
            obj.yDot = yDot;
            obj.v = v;
            obj.deltaF = deltaF;
            obj.deltaR = deltaR;
            obj.psiDot = psiDot;
            obj.psi = psi;
            obj.beta = beta;
            crash = true;
            optimumFound = false;
            itteration = 0;
            deltaFNew = obj.deltaF;
            score = sqrt((obj.x-obj.xTarget)^2 + (obj.y-obj.yTarget)^2);
            scoreVector = [];
            angleVector = [];
            for i =-30:1:30
                obj.x = x;
                obj.y = y;
                obj.xDot = xDot;
                obj.yDot = yDot;
                obj.v = v;
                obj.deltaF = deltaF;
                obj.deltaR = deltaR;
                obj.psiDot = psiDot;
                obj.psi = psi;
                obj.beta = beta;
                obj.deltaF = deg2rad(i);
                obj.simulate_step();
                score = sqrt((obj.x-obj.xTarget)^2 + (obj.y-obj.yTarget)^2);
                scoreVector(end+1) = score;
                angleVector(end+1) = obj.deltaF;
            end
            table = [scoreVector',angleVector'];
            table = sortrows(table);
            obj.x = x;
            obj.y = y;
            obj.xDot = xDot;
            obj.yDot = yDot;
            obj.v = v;
            obj.deltaF = table(1,2);
            obj.deltaR = deltaR;
            obj.psiDot = psiDot;
            obj.psi = psi;
            obj.beta = beta;
                itteration = 1;
            while crash
                obj.x = x;
                obj.y = y;
                obj.xDot = xDot;
                obj.yDot = yDot;
                obj.v = v;
                obj.deltaR = deltaR;
                obj.psiDot = psiDot;
                obj.psi = psi;
                obj.beta = beta;
                itteration = itteration+1;
                for i = 1:obj.predictionStep
                    obj.simulate_step();
                    [crash,~] = obj.check_crash();
                    if crash
                        if itteration > numel(table)/2;
                            obj.deltaF = table(numel(table)/2,2);
                            crash = false;
                            break
                        end
                        obj.deltaF = table(itteration,2);
                        break
                    end
                end
            end
            dF = obj.deltaF;
            dR = obj.deltaR;
            V = obj.v;
        end
        
        function [crashed,intersectPoint]= check_crash(obj)
            intersectPoint = [];
            rFront = sqrt((obj.height*obj.ratioT)^2+(obj.width/2)^2);
            rRear = sqrt((obj.height*(1-obj.ratioT))^2+(obj.width/2)^2);
            FLangle = atan((obj.width/2)/(obj.height*(obj.ratioT)));
            RLangle = pi - atan((obj.width/2)/(obj.height*(1-obj.ratioT)));
            RRangle = -RLangle;
            FRangle = -FLangle;
            FL = [obj.x + rFront * cos(FLangle+obj.psi); obj.y + rFront * sin(FLangle+obj.psi)];
            FR = [obj.x + rFront * cos(FRangle+obj.psi); obj.y + rFront * sin(FRangle+obj.psi)];
            RL = [obj.x + rRear  * cos(RLangle+obj.psi); obj.y + rRear  * sin(RLangle+obj.psi)];
            RR = [obj.x + rRear  * cos(RRangle+obj.psi); obj.y + rRear  * sin(RRangle+obj.psi)];
            vehicleBorder = [FL,FR,RR,RL,FL];
%             plot(vehicleBorder(1,:),vehicleBorder(2,:),'k')
%             pause(.1)    
            crashed = false;
            if isempty(vehicleBorder)
                return
            end
            %             vehicleLines = get_veh_lines(obj,vehicleBorder);
            %             for i = 1:length(vehicleLines)
            for j = 1:length(obj.knownEnviroment)
                %% go throught all lines (obstacles) and check if there is intersection between car and obsacles
                tempLine = obj.knownEnviroment{j};
                tempLineX = tempLine(1,:);
                tempLineY = tempLine(2,:);
                [xPoint,yPoint] = intersections(vehicleBorder(1,:),vehicleBorder(2,:),tempLineX,tempLineY);
                if ~isempty(xPoint)
                    crashed = true;
                    intersectPoint = [xPoint,yPoint];
                end
            end
            %             end
        end
        
        function simulate_step(obj)
            obj.beta = atan((obj.wheelBaseX*obj.ratioT*tan(obj.deltaR))+(obj.wheelBaseX*(1-obj.ratioT)*tan(obj.deltaF))/(obj.wheelBaseX));
            obj.psiDot = ((obj.v*cos(obj.beta))/obj.wheelBaseX)*(tan(obj.deltaF)-tan(obj.deltaR));
            obj.psi = obj.psi + obj.psiDot*obj.simStep;
            obj.yDot = obj.v*sin(obj.psi + obj.beta);
            obj.xDot = obj.v*cos(obj.psi + obj.beta);
            obj.x = obj.xDot*obj.simStep + obj.x;
            obj.y = obj.yDot*obj.simStep + obj.y;
        end
        
        function lines = get_known_space(obj)
            lines = {};
            for i = 1:length(obj.vehicle.sensors)
                currentSensor = obj.vehicle.sensors{i};
                for j = 1: length(currentSensor.lasers) - 1
                    if currentSensor.lasers{2,j} == currentSensor.rangeDistance
                        continue
                    end
                    point1 = currentSensor.lasers{1,j};
                    point2 = currentSensor.lasers{1,j+1};
                    lines{end+1} = [point1', point2'];
                end
            end
            obj.plot_what_veh_see(lines);
        end
        
        function plot_what_veh_see(obj,lines)
            for i = 1:numel(lines)
                hold on
                plot(lines{i}(1,:),lines{i}(2,:),'k','LineWidth', 1.2)
            end
            xlim(obj.vehicle.enviroment.xLim)
            ylim(obj.vehicle.enviroment.yLim)
            pause(.1)
        end
    end
    
end

