classdef enviroment < handle
    %ENVIROMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vehicle
        masterSwitch = 1;
        enviromentMap
        xLim
        yLim
        lines = {};
        lineVectors = {};
    end
    
    methods
        function obj = enviroment(vehicle_)
            obj.vehicle = vehicle_;
            obj = obj.gen_enviroment();
        end
        
        function obj = gen_enviroment(obj)
            switch obj.masterSwitch
                case 1 
                    obj.xLim = [-10.5,10.5];
                    obj.yLim = [-10.5,10.5];
                    %% end of world
                    obj.lines{end+1} = [-10,10;-10,-10];
                    obj.lines{end+1} = [-10,-10;10,-10];
                    obj.lines{end+1} = [10,10;10,-10];
                    obj.lines{end+1} = [-10,10;10,10];
                case 2
                    
                otherwise
                    
            end 
            for i = 1:numel(obj.lines)
                obj.lineVectors{end+1,1} = obj.lines{i}(1,:);
                obj.lineVectors{end,2} = obj.lines{i}(2,:) - obj.lines{i}(1,:);
            end
        end
    end
    
end

