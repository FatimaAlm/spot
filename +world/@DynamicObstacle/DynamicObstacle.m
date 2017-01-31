classdef DynamicObstacle < world.Obstacle & globalPck.Dynamic
    % Obstacle - class for describing dynamic obstacles
    %
    % Syntax:
    %   object constructor: obj = DynamicObstacle(timeInterval, position, orientation,
    %           width, length, velocity, v_max, a_max, time, inLane)
    %
    % Inputs:
    %   position - position of the obstacle's center in UTM coordinates
    %   orientation - orientation of the obstacle in radians
    %   width - width of the obstacle in m
    %   length - length of the obstacle in m
    %   velocity - scalar velocity of the obstacle in m/s
    %   acceleration - scalar acceleration of the obstacle in m/s^2
    %   v_max - maximum velocity of the obstacle in m/s
    %   a_max - maximum absolute acceleration of the obstacle in m/s^2
    %   time - current time of the obstacle
    %   inLane - lane, in which the obstacle is located in
    %
    % Outputs:
    %   obj - generated object
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    
    % Author:       Markus Koschi
    % Written:      27-October-2016
    % Last update:
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (SetAccess = protected, GetAccess = public)
        % inherited from parent classes:
        % id = [];
        % position = [];
        % orientation = [];
        % width = [];
        % length = [];
        % inLane = world.Lane.empty();
        % time = 0;
        % occupancy = prediction.Occupancy.empty();
        
        velocity = [];
        acceleration = [];
        v_max = 83.3333; % = 300km/h
        a_max = 5; %inf;
        trajectory = globalPck.Trajectory.empty();
        
        % constraints for occupancy prediction:
        % parameter, how many percent above the speed limit the obstacle's
        % speed can get
        speedingFactor = prediction.Occupancy.INITIAL_SPEEDING_FACTOR;
        % constraint C3: backward driving is not allowed by default, i.e.
        % constraint3 is true
        constraint3 = true;
        % constraint C5: changing and crossing lanes is forbidden unless 
        % allowed by traffic regulations. if traffic participant is
        % following the rules, constraint5 is true
        constraint5 = true; %WARNING: false might not work yet
    end
    
    methods
        % class constructor
        function obj = DynamicObstacle(timeInterval, position, orientation,...
                width, length, inLane, velocity, acceleration, v_max, a_max, time)
            % instantiate parent class
            if nargin >= 6
                super_args = {[], [], width, length, inLane};
            elseif nargin >= 5
                super_args = {[], [], width, length};
            else
                super_args = {};
            end
            obj = obj@world.Obstacle(super_args{:});
            
            % instantiate trajectory                       
            if nargin >= 8
                obj.trajectory = globalPck.Trajectory(timeInterval, position,...
                    orientation, velocity, acceleration); 
            elseif nargin >= 3
                obj.trajectory = globalPck.Trajectory(timeInterval, position,...
                    orientation); 
            end
                     
            % Update current position, orientation, velocity and
            % acceleration
            if nargin >= 11
                obj.update(time);
            elseif nargin >= 3 && ~isempty(timeInterval)
                obj.update(timeInterval.ts);
            end
           
            % set maximal velocity and acceleration of obstacle
            if nargin >= 10
                obj.v_max = v_max;
                obj.a_max = a_max;
            end
        end
        
        % methods in seperate files:
        % set methods
        set(obj, propertyName, propertyValue)
        setTrajectory(obj, trajectory)
        
        % Dynamic class methods
        step(obj)
        update(varargin)
        
        % visualization methods
        disp(obj)
        % plot(varargin) % implemented in superclass Obstacle
    end
end
