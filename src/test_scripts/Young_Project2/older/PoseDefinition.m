classdef PoseDefinition
    properties 
        x   % pose x position
        y   % pose y position
        z   % pose z position
        h   % pose heading
    end
    
    methods
        function obj = PoseDefinition(x, y, z, x_o, y_o, z_o, w)
            obj.x = x;
            obj.y = y;
            obj.z = z;
            switch nargin
                case 7
                    angles = quat2eul([w x_o y_o z_o]); %%%% May need to swap Y/X % Default format ZYX 
                    obj.h = wrapTo180(rad2deg(angles(1)));
                otherwise
                    obj.h = x_o; % assumes value in fourth spot is heading
            end
        end
    end
end