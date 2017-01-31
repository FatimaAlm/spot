classdef Vehicle < world.DynamicObstacle
    % Obstacle - class for describing static obstacles
    %
    % Syntax:
    %   object constructor: obj = Vehicle(timeInterval, position, orientation,...
    %           width, length, inLane, velocity, acceleration, v_max, a_max,...
    %           time, v_s, power_max)
    %
    % Inputs:
    %   timeInterval - time interval of defined trajectory
    %   position - position of the obstacle's center in UTM coordinates
    %   orientation - orientation of the obstacle in radians
    %   width - width of the obstacle in m
    %   length - length of the obstacle in m
    %   inLane - lane, in which the obstacle is located in
    %   velocity - scalar velocity of the obstacle in m/s
    %   acceleration - scalar acceleration of the obstacle in m/s^2
    %   v_max - maximum velocity of the obstacle in m/s
    %   a_max - maximum absolute acceleration of the obstacle in m/s^2
    %   time - current time of the obstacle
    %   v_s - switching velocity in m/s (modeling limited engine power)
    %   power_max - maximmum power for accelerating (currently unused)
    %
    % Outputs:
    %   obj - generated object
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    
    % Author:       Markus Koschi
    % Written:      26-October-2016
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
        % velocity = [];
        % acceleration = [];
        % v_max = inf;
        % a_max = inf;
        % time = 0;
        % trajectory = globalPck.Trajectory.emtpy();
        % occupancy = prediction.Occupancy.empty();
        
        v_s = 30; %inf;
        power_max = inf;
    end
    
    methods
        % class constructor
        function obj = Vehicle(timeInterval, position, orientation, width, length, ...
                inLane, velocity, acceleration, v_max, a_max, time, v_s, power_max)
            % instantiate parent class
            if nargin >= 11
                super_args = {timeInterval, position, orientation, width, length, inLane, velocity, acceleration, v_max, a_max, time};
            elseif nargin >= 10
                 super_args = {timeInterval, position, orientation, width, length, inLane, velocity, acceleration, v_max, a_max};
            elseif nargin >= 8
                 super_args = {timeInterval, position, orientation, width, length, inLane, velocity, acceleration};
            elseif nargin >= 6
                super_args = {timeInterval, position, orientation, width, length, inLane};
            elseif nargin >= 5
                super_args = {timeInterval, position, orientation, width, length};
            elseif nargin >= 3
                super_args = {timeInterval, position, orientation};
            else
                super_args = {};
            end
            obj = obj@world.DynamicObstacle(super_args{:});
            
            % set vehicle properties
            if nargin == 13
                obj.v_s = v_s;
                obj.power_max = power_max;
            end
            
            % old code:
            % % add missing parameters to each car
            % for i=1:length(cars);
            %     if isempty(cars(i).a_max)
            %         cars(i).a_max = 5;
            %     end
            %     if isempty(cars(i).v_max)
            %         cars(i).v_max = 50;
            %     end
            %     if isempty(cars(i).v_s)
            %         cars(i).v_s = 30;
            %     end
            % end
        end
        
        % methods in seperate files:
        % set function
        set(obj, propertyName, propertyValue)
        
        % Dynamic class methods
        step(obj)
        update(varargin)
        
        % visualization methods
        disp(obj)
        % plot(varargin) % implemented in superclass Obstacle
    end
end

%------------- END CODE --------------