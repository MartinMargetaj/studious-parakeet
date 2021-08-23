classdef brain < handle
    %BRAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensors = {};
        vehicle
        obstacles
        simulatedVehicle
        knownEnviroment
        predictionStep
    end
    
    methods
        function obj = brain(sensors_,vehicle_)
            obj.sensors = sensors_;
            obj.vehicle = vehicle_;
        end
        
        function [dF,dR,V] = ask_what_next(obj)
            obj.knownEnviroment = obj.get_known_space();
            crash = true;
            obj.simulatedVehicle = obj.vehicle;
            while crash
                for i = 1:obj.predictionStep
                    obj.simulate_step();
                    [crash,~] = obj.check_crash();
                    if crash
                        obj.simulatedVehicle.v = 0;
                    end
                end
            end
            dF = obj.simulatedVehicle.deltaF;
            dR = obj.simulatedVehicle.deltaR;
            V = obj.simulatedVehicle.v;
        end
        
        function [crashed,intersectPoint]= check_crash(obj)
            intersectPoint = [];
            rFront = sqrt((obj.simulatedVehicle.height*obj.simulatedVehicle.ratioT)^2+(obj.simulatedVehicle.width/2)^2);
            rRear = sqrt((obj.simulatedVehicle.height*(1-obj.simulatedVehicle.ratioT))^2+(obj.simulatedVehicle.width/2)^2);
            FLangle = atan((obj.simulatedVehicle.width/2)/(obj.simulatedVehicle.height*(obj.simulatedVehicle.ratioT)));
            RLangle = pi - atan((obj.simulatedVehicle.width/2)/(obj.simulatedVehicle.height*(1-obj.simulatedVehicle.ratioT)));
            RRangle = -RLangle;
            FRangle = -FLangle;
            FL = [obj.simulatedVehicle.x + rFront * cos(FLangle+obj.simulatedVehicle.psi); obj.simulatedVehicle.y + rFront * sin(FLangle+obj.simulatedVehicle.psi)];
            FR = [obj.simulatedVehicle.x + rFront * cos(FRangle+obj.simulatedVehicle.psi); obj.simulatedVehicle.y + rFront * sin(FRangle+obj.simulatedVehicle.psi)];             
            RL = [obj.simulatedVehicle.x + rRear  * cos(RLangle+obj.simulatedVehicle.psi); obj.simulatedVehicle.y + rRear  * sin(RLangle+obj.simulatedVehicle.psi)];
            RR = [obj.simulatedVehicle.x + rRear  * cos(RRangle+obj.simulatedVehicle.psi); obj.simulatedVehicle.y + rRear  * sin(RRangle+obj.simulatedVehicle.psi)];
            vehicleBorder = [FL,FR,RR,RL,FL];
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
                    [x,y] = intersections(vehicleBorder(1,:),vehicleBorder(2,:),tempLineX,tempLineY);
                    if ~isempty(x)
                        crashed = true;
                        intersectPoint = [x,y];
                    end
                end
%             end
        end
        
        function simulate_step(obj)
            obj.simulatedVehicle.beta = atan((obj.simulatedVehicle.wheelBaseX*obj.simulatedVehicle.ratioT*tan(obj.simulatedVehicle.deltaR))+(obj.simulatedVehicle.wheelBaseX*(1-obj.simulatedVehicle.ratioT)*tan(obj.simulatedVehicle.deltaF))/(obj.simulatedVehicle.wheelBaseX));
            obj.simulatedVehicle.psiDot = ((obj.simulatedVehicle.v*cos(obj.simulatedVehicle.beta))/obj.simulatedVehicle.wheelBaseX)*(tan(obj.simulatedVehicle.deltaF)-tan(obj.simulatedVehicle.deltaR));
            obj.simulatedVehicle.psi = obj.simulatedVehicle.psi + obj.simulatedVehicle.psiDot*obj.simulatedVehicle.simStep;
            obj.simulatedVehicle.yDot = obj.simulatedVehicle.v*sin(obj.simulatedVehicle.psi + obj.simulatedVehicle.beta);
            obj.simulatedVehicle.xDot = obj.simulatedVehicle.v*cos(obj.simulatedVehicle.psi + obj.simulatedVehicle.beta);
            obj.simulatedVehicle.x = obj.simulatedVehicle.xDot*obj.simulatedVehicle.simStep + obj.simulatedVehicle.x;
            obj.simulatedVehicle.y = obj.simulatedVehicle.yDot*obj.simulatedVehicle.simStep + obj.simulatedVehicle.y;
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
        end
    end
    
end

