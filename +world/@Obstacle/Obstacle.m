classdef Obstacle < matlab.mixin.Heterogeneous & handle
    % Obstacle - class for describing static and dynamic obstacles
    % (Inherits from matlab.mixin.Heterogeneous to combine instances of the
    % subclasses into heterogeneous arrays)
    %
    % Syntax:
    %   object constructor: obj = Obstacle(id, position, orientation, width, length, inLane)
    %
    % Inputs:
    %   id - unique id of the obstacle
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
    % Written:      25-October-2016
    % Last update:
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (SetAccess = protected, GetAccess = public)
        id = [];
        position = [];
        orientation = [];
        width = [];
        length = [];
        inLane = world.Lane.empty();
        occupancy = prediction.Occupancy.empty();
        
        color = [rand, rand, rand];
    end
    
    methods
        % class constructor
        function obj = Obstacle(position, orientation, width, length, inLane)
            obj.id = obj.addId();
            
            if nargin == 5 && isa(inLane, 'world.Lane')
                obj.inLane = inLane;
            end
            if nargin >= 4
                obj.position = position;
                obj.orientation = orientation;
                obj.width = width;
                obj.length = length;
                obj.occupancy = prediction.Occupancy(); %obj.occupancy = prediction.Occupancy(obj);
            end
             obj.color = globalPck.PlotProperties.COLOR_OBSTACLES(...
                 mod(obj.id, numel(globalPck.PlotProperties.COLOR_OBSTACLES))+1);
        end
        
        % set methods
        function setInLane(obj, inLane)
            if isa(inLane, 'world.Lane')
                obj.inLane = inLane;
            else
                error(['Input argument is not an instance of the class '...
                    'Lane. Error in world.Obstacle/setInLane']);
            end
        end
        % % function setOccupancy(obj, occupancy)
        % %     obj.occupancy = occupancy;
        % % end
        
        % methods in seperate files:
        % occupancy
        computeOccupancyForObstacle(obj, map, timeInterval)
        
        % visualization methods
        disp(obj)
        plot(varargin) % argin: obj, timeInterval
    end
    
    methods (Static)
        function id = addId(init)
            % Creates unique ids
            persistent id_count;
            if nargin==1
                assert(isnumeric(init) && round(init) == init && numel(init)==1);
                id_count = int32(init);
            end
            if (numel(id_count)==0)
                id_count = 0;
            end
            id_count = id_count + 1;
            id = id_count;
        end
        
        % static methods in seperate files
        [obstacles_dynamic, vehicles, obstacles_static] = createObstaclesFromXML(obstacles_XML, lanelets);
    end
    
    methods (Abstract)
        % ?
    end
end

%------------- END CODE --------------