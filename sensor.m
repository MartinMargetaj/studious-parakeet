classdef sensor < handle
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position
        rangeDistance
        positionAngle
        rangeAngle
        sections
        vehicle
        enviroment
        actSnsrPos
        lasers = {};
        color
    end
    
    methods
        function obj = sensor(vehicle_, enviroment_, pos,type, angle)
            obj.vehicle = vehicle_;
            obj.enviroment = enviroment_;
            switch type
                case 'lowRange'
                    obj.positionAngle = angle;
                    obj.rangeDistance = 2;
                    obj.rangeAngle = pi;
                    obj.sections = 3;
                    obj.color = 'g';
                case 'highRange'
                    obj.positionAngle = 0;
                    obj.rangeDistance = 10;
                    obj.rangeAngle = 2*pi;
                    obj.sections = 18;
                    obj.color = 'c';
            end
            obj.position = pos;
        end
        
        function obj = sim_lasers(obj)
            obj.lasers = {};
            [Theta, rSensor] = cart2pol(obj.position(1),obj.position(2));
            obj.actSnsrPos = [obj.vehicle.x + rSensor*cos(obj.vehicle.psi+Theta),obj.vehicle.y + rSensor*sin(obj.vehicle.psi+Theta)];
            oneTickAngle = obj.rangeAngle/obj.sections;
            for i = -obj.sections/2:obj.sections/2
                currentLsrAngle = i*oneTickAngle;
                lsrEndPoint = [obj.actSnsrPos(1) + obj.rangeDistance * cos(obj.vehicle.psi + currentLsrAngle + obj.positionAngle), obj.actSnsrPos(2) + obj.rangeDistance * sin(obj.vehicle.psi + currentLsrAngle + obj.positionAngle)];
                lsrEndPoint = obj.vehicle.crashDetn.get_end_point(obj.actSnsrPos,lsrEndPoint);
                obj.lasers{end+1} = lsrEndPoint;
            end
%             plot(obj.actSnsrPos(1),obj.actSnsrPos(2),'xr')
        end
    end
end

