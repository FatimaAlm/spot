classdef Map < globalPck.Dynamic
    % Map - class for holding all objects in the environment of the ego vehicle
    %
    % Syntax:
    %  object constructor: obj = Map(varargin)
    %
    % Inputs:
    %   varargin
    %
    % Outputs:
    %   obj - generated object
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    %
    % See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic Participants on
    % Arbitrary Road Networks, III. A. Road Network Representation
    
    % Author:       Markus Koschi
    % Written:      02-November-2016
    % Last update:  02-Dezember-2016
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (SetAccess = protected, GetAccess = public)
        % inherited from abstract class Dynamic:
        % time = 0;
        
        lanes = world.Lane.empty();
        obstacles = world.Obstacle.empty();
        egoVehicle = world.Vehicle.empty();
    end
    
    methods
        % class constructor
        function obj = Map(varargin)
            if nargin >= 1 && isa(varargin{1}, 'world.Lane')
                % set lanes
                obj.lanes = varargin{1};
                
                % set obstacles
                if nargin >= 2 && isa(varargin{2}, 'world.Obstacle')
                    obj.obstacles = varargin{2};
                end
                
                % set ego vehicle
                if nargin == 3 && isa(varargin{3}, 'world.Vehicle')
                    obj.egoVehicle = varargin{3};
                end
                
                % map is given as xml data:
            elseif nargin == 1 && ischar(varargin{1})
                % check for type of file
                % [pathstr, name, ext] = fileparts(filename)
                [~, name, ext] = fileparts(varargin{1});
                if strcmp(ext,'.osm') || strcmp(ext,'.xml')
                    
                    % read input file
                    [lanelets_XML, obstacles_XML, egoVehicle_XML, adjacencyGraph_XML] = input.readXMLFile(varargin{1});
                    
                    % create lanelet objects
                    lanelets = world.Lanelet.createLaneletsFromXML(lanelets_XML, adjacencyGraph_XML);
                    
                    % create lanes from lanelets
                    obj.lanes = world.Lane.createLanesFromLanelets(lanelets);
                    
                    % create obstacles
                    [obstacles_dynamic, vehicles, obstacles_static] = world.Obstacle.createObstaclesFromXML(obstacles_XML, obj.lanes);
                    obj.addObstacle([obstacles_dynamic, vehicles, obstacles_static]);
                    
                    % hard coded obstacles for quick changes of the traffic
                    % participants' initial configurations
                    % obj = Vehicle(timeInterval, position, orientation, width, length, inLane, velocity, acceleration, v_max, a_max, time, v_s, power_max)
                    if strcmp(name,'Intersection_Leopold_Hohenzollern_1')
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,4), -1.841, 2.0, 4.8, obj.lanes(1), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(5).center.vertices(:,7), 1.300, 2.0, 4.8, obj.lanes(5), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(12).center.vertices(:,4), -0.2685, 2.0, 4.8, [obj.lanes(12), obj.lanes(13)], 50/3.6, 1, 50, 8, 0, 15, 0));
                    elseif strcmp(name,'Intersection_Leopold_Hohenzollern_2')
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(3).center.vertices(:,7), -1.841, 2.0, 4.8, obj.lanes(3), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,16), pi-0.2685, 2.0, 4.8, obj.lanes(1), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(5).center.vertices(:,9), 1.300, 2.0, 4.8, obj.lanes(5), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(5).center.vertices(:,2), 1.300, 2.0, 4.8, obj.lanes(5), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(6).center.vertices(:,15), 1.300, 2.0, 4.8, obj.lanes(6), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(7).center.vertices(:,12), -0.2685, 2.0, 4.8, obj.lanes(7), 50/3.6, 1, 50, 8, 0, 15, 0));
                    elseif strcmp(name,'Intersection_Leopold_Hohenzollern_3')
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,4), -1.841, 2.0, 4.8, obj.lanes(1), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,7), -1.841, 2.0, 4.8, obj.lanes(1), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(2).center.vertices(:,5), -1.841, 2.0, 4.8, obj.lanes(2), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(2).center.vertices(:,12), -1.841, 2.0, 4.8, obj.lanes(2), 50/3.6, 1, 50, 8, 0, 15, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(2).center.vertices(:,17), -1.841, 2.0, 4.8, obj.lanes(2), 50/3.6, 1, 50, 8, 0, 15, 0));                        
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,15), pi-0.2685, 2.0, 4.8, obj.lanes(1), 50/3.6, 1, 50, 8, 0, 15, 0));
                    elseif strcmp(name,'Example_straight_2lanes')
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,6), 0.0332, 2.0, 4.8, obj.lanes(1), 50/3.6, 0, 50, 8, 0, 15, 0)); %40/3.6
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(2).center.vertices(:,8), pi+0.0332, 2.0, 4.8, obj.lanes(2), 50/3.6, 0, 50, 8, 0, 15, 0));
                    elseif strcmp(name,'A9_Freimann_1_direction')
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(1).center.vertices(:,5), 1.265, 2.0, 4.0, obj.lanes(1), 30, 0, 50, 5, 0, 30, 0));
                        obj.addObstacle(world.Vehicle(globalPck.TimeInterval(0,0.5,0.5,0), obj.lanes(3).center.vertices(:,7), 1.265, 2.0, 4.0, obj.lanes(1), 30, 0, 50, 5, 0, 30, 0));
                    end
                                        
                    % create ego vehicle
                    [~, obj.egoVehicle, ~] = world.Obstacle.createObstaclesFromXML(egoVehicle_XML, obj.lanes);
                else
                    error('Invalid input file. Error in world.Map');
                end
            end
        end
        
        % function to add obstacle(s) as a map property
        function addObstacle(obj, obstacle)
            if isa(obstacle, 'world.Obstacle')
                for i = 1:numel(obstacle)
                    obj.obstacles(end+1) = obstacle(i);
                end
            else
                error(['Input argument is not an instance of the class '...
                    'Obstacle. Error in world.Map/addObstacle']);
            end
        end
        
        % methods in seperate files:
        % Dynamic class methods
        step(obj)
        update(varargin)
        
        % visualization methods
        disp(obj)
        plot(varargin) % argin: obj, timeInterval
    end
end

%------------- END CODE --------------