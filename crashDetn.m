classdef crashDetn < handle
    %CRASHDETN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vehicle
        enviroment
        plotter
    end
    
    methods
        function obj = crashDetn(vehicle_)
            obj.vehicle = vehicle_;
            obj.enviroment = vehicle_.enviroment;
            obj.plotter = vehicle_.plotter;
        end
        
        function [crashed, intersectPoint]= vehicle_crashed(obj)
            crashed = false;
            intersectPoint = [];
            if isempty(obj.plotter.vehicleBorder)
                return
            end
%             vehicleLines = get_veh_lines(obj,obj.plotter.vehicleBorder);
%             for i = 1:length(vehicleLines)
                for j = 1:length(obj.enviroment.lines)
                    %% go throught all lines (obstacles) and check if there is intersection between car and obsacles
                    tempLine = obj.enviroment.lines{j};
                    tempLineX = tempLine(1,:);
                    tempLineY = tempLine(2,:);
                    [x,y] = intersections(obj.plotter.vehicleBorder(1,:),obj.plotter.vehicleBorder(2,:),tempLineX,tempLineY);
                    if ~isempty(x)
                        crashed = true;
                        intersectPoint = [x,y];
                    end
                end
%             end
        end
        
        function endPoint = get_end_point(obj, point1, point2)
            endPoint = point2;
            for j = 1:length(obj.enviroment.lines)
                %% go through all lines(obsatcles) and find endpoint
                tempLine = obj.enviroment.lines{j};
                tempLineX = tempLine(1,:);
                tempLineY = tempLine(2,:);
                [x,y] = intersections([point1(1), point2(1)],[point1(2), point2(2)],tempLineX,tempLineY);
                if ~isempty(x)
                    endPoint = [x,y];
                end
            end
            
        end
%         function lines = get_veh_lines(~,borders)
%             lines = {};
%             for i = 1:length(borders)-1
%                 lines{end+1,1} = borders(:,i);
%                 lines{end,2} = borders(:,i+1) - borders(:,i);
%             end
%         end
    end
    
end

