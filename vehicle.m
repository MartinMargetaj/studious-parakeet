classdef vehicle < handle
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        plotter
        enviroment
        crashDetn
        sensors = {};
        brain
        x = 0%9;
        y = 0%-7.2;
        xDot = 0;
        yDot = 0;
        v = 3;
        deltaF = deg2rad(0);
        deltaR = deg2rad(0);
        psiDot = 0; 
        psi = pi/2;
        beta = 0;
        width = 1.760;
        height = 4.585;
        wheelBaseX = 2.635;
        wheelBaseY = 1.530;
        ratioT = 0.5;
        simStep = 0.01;
        predictionLength = 1000;
    end
    
    methods
        function obj = vehicle
            obj.plotter = plotter(obj);
            obj.enviroment = enviroment(obj);
            obj.crashDetn = crashDetn(obj);
            obj.sensors{end + 1} = sensor(obj,obj.enviroment, [obj.height/2, 0],'lowRange',0);
            obj.sensors{end + 1} = sensor(obj,obj.enviroment, [-obj.height/2, 0],'lowRange',pi);
            obj.sensors{end + 1} = sensor(obj,obj.enviroment, [obj.height/2, obj.width/2],'lowRange',pi/2);
            obj.sensors{end + 1} = sensor(obj,obj.enviroment, [obj.height/2, -obj.width/2],'lowRange',-pi/2);
%             obj.sensors{end + 1} = sensor(obj,obj.enviroment, [-obj.height/2, obj.width/2],'lowRange',pi/2);
%             obj.sensors{end + 1} = sensor(obj,obj.enviroment, [-obj.height/2, -obj.width/2],'lowRange',-pi/2);
            obj.sensors{end + 1} = sensor(obj,obj.enviroment, [0, 0],'highRange',0);
            obj.brain = brain(obj.plotter,obj.enviroment,obj.crashDetn,obj.sensors,obj);
        end
        
        function obj = sim_step(obj)
            for i =1:numel(obj.sensors)
                obj.sensors{i} =  obj.sensors{i}.sim_lasers;
            end
            [dF,dR,dV] = obj.brain.ask_what_next;
%             obj.deltaF = dF;
%             obj.deltaR = dR;
%             obj.v = dV;
            if obj.v < 5
                obj.beta = atan((obj.wheelBaseX*obj.ratioT*tan(obj.deltaR))+(obj.wheelBaseX*(1-obj.ratioT)*tan(obj.deltaF))/(obj.wheelBaseX));
                obj.psiDot = ((obj.v*cos(obj.beta))/obj.wheelBaseX)*(tan(obj.deltaF)-tan(obj.deltaR));
                obj.psi = obj.psi + obj.psiDot*obj.simStep;
                obj.yDot = obj.v*sin(obj.psi + obj.beta);
                obj.xDot = obj.v*cos(obj.psi + obj.beta);
                obj.x = obj.xDot*obj.simStep + obj.x;
                obj.y = obj.yDot*obj.simStep + obj.y;
            else
            end
        end
        
        function prediction = sim_fure_steps(obj)
            prediction = zeros(2,obj.predictionLength);
            betaFuture = atan((obj.wheelBaseX*obj.ratioT*tan(obj.deltaR))+(obj.wheelBaseX*(1-obj.ratioT)*tan(obj.deltaF))/(obj.wheelBaseX));
            psiDotFuture = ((obj.v*cos(betaFuture))/obj.wheelBaseX)*(tan(obj.deltaF)-tan(obj.deltaR));
            psiFuture = obj.psi;
            xFuture = obj.x;
            yFuture = obj.y;
            for i = 1:obj.predictionLength
                psiFuture = psiFuture + psiDotFuture*obj.simStep;
                yDotFuture = obj.v*sin(psiFuture + betaFuture);
                xDotFuture = obj.v*cos(psiFuture + betaFuture);
                xFuture = xDotFuture*obj.simStep + xFuture;
                yFuture = yDotFuture*obj.simStep + yFuture;
                prediction(1,i) = xFuture;
                prediction(2,i) = yFuture;
            end
        end
    end
end

