classdef Trajectory < handle
    % Trajectory - class to describe the configuration of an dynamic
    % obstacle over time
    %
    % Syntax:
    %   obj = Trajectory(timeInterval, position, velocity, acceleration, inLane)
    %
    % Inputs: (all row vectors with the same length)
    %   position - position of the obstacle's center in UTM coordinates [x;y]
    %   orientation - orientation of the obstacle in radians
    %   velocity - velocity of the obstacle in m/s
    %   acceleration - acceleration of the obstacle in m/s^2 
    %
    % Outputs:
    %   obj - generated object
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    
    % Author:       Markus Koschi
    % Written:      15-November-2016
    % Last update:
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (SetAccess = protected, GetAccess = public)
        timeInterval = globalPck.TimeInterval.empty();
        position = [];
        orientation = [];
        velocity = [];
        acceleration = [];
        %inLane = world.Lane.empty();
    end
    
    methods
         % class constructor
        function obj = Trajectory(timeInterval, position, orientation, ...
                                  velocity, acceleration)
            if nargin >= 3
                obj.timeInterval = timeInterval;
                obj.position = position;
                obj.orientation = orientation;
            end
            if nargin >= 4
                obj.velocity = velocity;
            end
            if nargin == 5
                obj.acceleration = acceleration;
            end
        end
        
        % get methods
        function position = getPosition(obj,t)
            if(~isempty(obj.position))
                position = obj.position(:,obj.timeInterval.getIndex(t));
            else
                position = [];
            end
        end
        
        function orientation = getOrientation(obj,t)
            if(~isempty(obj.orientation))
                orientation = obj.orientation(obj.timeInterval.getIndex(t));
            else
                orientation =  [];
            end
        end
        
        function velocity = getVelocity(obj,t)
            if(~isempty(obj.velocity))
                velocity = obj.velocity(:,obj.timeInterval.getIndex(t));
            else
                velocity = [];
            end
        end
        
        function acceleration = getAcceleration(obj,t)
            if(~isempty(obj.acceleration))
                acceleration = obj.acceleration(:,obj.timeInterval.getIndex(t));
            else
                acceleration = [];
            end
        end   
        
        % visualization methods
        disp(obj)
        plot(varargin) % argin: obj, timeInterval
    end
end

%------------- END CODE --------------