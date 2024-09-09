classdef PIDcontroller
    properties 
        kp
        ki
        kd
        integral = 0
        previous_error = 0
    end
    
    methods
        function obj = PIDcontroller(kp, ki, kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
        end

        function [output, error] = update(obj, current_value, desired_value, dt)
            error = desired_value - current_value;
            obj.integral = obj.integral + error * dt;
            derivative = (error - obj.previous_error) / dt;
            output = (obj.kp * error) + (obj.ki * obj.integral) + (obj.kd * derivative);
            obj.previous_error = error;
        end

        function [output, error] = updateAngle(obj, current_angle, desired_angle, dt)
            unwrapped_error = desired_angle - current_angle;
            error = wrapTo180(unwrapped_error);
            obj.integral = obj.integral + error * dt;
            derivative = (error - obj.previous_error) / dt;
            output = (obj.kp * error) + (obj.ki * obj.integral) + (obj.kd * derivative);
            obj.previous_error = error;
        end
    end
end