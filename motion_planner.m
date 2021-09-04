classdef motion_planner
    %MOTION_PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        map
        xFinal
        yFinal
        psiFinal
        xStart
        yStart
        psiStart
    end
    
    methods
        function obj = motion_planner(walls_,xFinal_, yFinal_, psiFinal_, xStart_, yStart_, psiStart_)
            obj.map = obj.generate_map(walls_);
            obj.xFinal = xFinal_;
            obj.yFinal = yFinal_;
            obj.psiFinal = psiFinal_;
            obj.xStart = xStart_;
            obj.yStart = yStart_;
            obj.psiStart = psiStart_;
        end
        
        function map = generate_map(obj,walls)
            mapSize = 50;
            map = zeros(mapSize);
            for i = 1:numel(walls)
                k = (walls{i}(2,1)-walls{i}(2,2))/(walls{i}(1,1)-walls{i}(1,2));
                if k == -inf || k == inf
                    k = 0;
                else
                v = walls{i}(2,1) - k*walls{i}(1,1);
                end
                for j = 0:mapSize
                    yTemp = k*j + v;
                    if yTemp > mapSize-1
                        break
                    end
                    yTempOne = ceil(yTemp);
                    yTempTwo = floor(yTemp);
                    map(j+1,yTempOne+1) = 1;
                    map(j+1,yTempTwo+1) = 1;
                end
            end
        end
        
        function points = basic_trajectory(obj)
            points = [];
            disp('TBD')
            
        end
    end
    
end

