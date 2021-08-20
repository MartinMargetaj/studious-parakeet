classdef brain
    %BRAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        plotter
        enviroment
        crashDetn
        sensors = {};
        vehicle
        obstacles
    end
    
    methods
        function obj = brain(plotter_,enviroment_,crashDetn_,sensors_,vehicle_)
            obj.plotter = plotter_;
            obj.enviroment = enviroment_;
            obj.crashDetn = crashDetn_;
            obj.sensors = sensors_;
            obj.vehicle = vehicle_;
        end
        
        function [dF,dR,V] = ask_what_next(obj)
            dF = 0;
            dR = 0;
            V = 0;
            
        end
    end
    
end

