classdef StaticObstacle < world.Obstacle
    % Obstacle - class for describing static obstacles
    %
    % Syntax:
    %   object constructor: obj = StaticObstacle(position, orientation, width, length, inLane)
    %
    % Inputs:
    %   position - position of the obstacle's center in UTM coordinates
    %   orientation - orientation of the obstacle in radians
    %   width - width of the obstacle in m
    %   length - length of the obstacle in m
    %   inLane - lane, in which the obstacle is located in
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
        % inherited from abstract class Obstacle:
        % id = [];
        % position = [];
        % orientation = [];
        % width = [];
        % length = [];
        % inLane = world.Lane.empty();
        % occupancy = prediction.Occupancy.empty();
    end
    
    methods
        % class constructor
        function obj = StaticObstacle(position, orientation, width, length, inLane)
            % instantiate parent class
            if nargin == 5
                super_args = {position, orientation, width, length, inLane};
            elseif nargin == 4
                super_args = {position, orientation, width, length};
            else
                super_args = {0};
            end
            obj = obj@world.Obstacle(super_args{:});
        end
        
        % methods in seperate files:
        disp(obj)
        % plot(varargin) % implemented in superclass Obstacle
    end
end
