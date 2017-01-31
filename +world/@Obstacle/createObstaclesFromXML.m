function [obstacles_dynamic, vehicles, obstacles_static] = createObstaclesFromXML(obstacles_XML, lanes)
%CREATEOBSTACLESFROMXML This function creates Obstacles objects from
%                       readXMLFile output data
%   Syntax:
%   [obstacles, obstacles_dynamic, vehicles, obstacles_static] = createObstaclesFromXML(obstacles_XML,lanes)
%
%   Inputs:
%   obstacles_XML - structure of obstacles
%           .id
%           .shape
%               .id
%               .length
%               .width
%           .trajectory
%               .id
%               .nodes
%                   .id
%                   .x
%                   .y
%                   .orientation
%                   .velocity
%                   .acceleration
%                   .time
%               .type
%           .role
%           .type
%   lanelets - structure of Lanelet Objects
%           .id
%           .leftBorderVertices
%           .rightBorderVertices
%           .speedLimit
%           .predecessorLanelets
%           .successorLanelets
%           .successorLanelets
%           .adjacentLeft
%           .adjacentRight
%           .centerVertices
% 
%
%   Outputs:
%       obstacles_dynamic - array of dynamic obstacle objects
%       vehicles - array of vehicle objects
%       obstacles_static - array of static obstacle objects
% 
% 
%           -without initial position(except of static obstacles).
%           -findLaneletByPosition muss noch implementiert werden.
%
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author:       Lukas Braunstorfer
% Written:      25-Dezember-2016
% Last update:
%
% Last revision:---



%------------- BEGIN CODE --------------

% %initialize outputs
numObstacles = length(obstacles_XML);
obstacles_dynamic = world.DynamicObstacle.empty();
vehicles = world.Vehicle.empty();
obstacles_static = world.StaticObstacle.empty();

%create counters for counting the outputdimensions.
vehicle_count = 1;
obstacle_dynamic_count = 1;
obstacle_static_count = 1;
timeinterval_empty = globalPck.TimeInterval.empty();

if nargin~=2
    warning('not adequat input');
end


for i=1:numObstacles
    switch obstacles_XML(i).role %differentiating between dynamic and static
        
        case 'dynamic'
            
            if(length(obstacles_XML(i).trajectory.nodes)==1) %in Case of single node as trajectory
                timeinterval = globalPck.TimeInterval(obstacles_XML(i).trajectory.nodes(1).time, 0, obstacles_XML(i).trajectory.nodes(1).time);
                position = [obstacles_XML(i).trajectory.nodes(1).x;obstacles_XML(i).trajectory.nodes(1).y];
                orientation = obstacles_XML(i).trajectory.nodes(1).orientation;
                velocity = obstacles_XML(i).trajectory.nodes.velocity;
                acceleration = obstacles_XML(i).trajectory.nodes.acceleration;
                trajectory = globalPck.Trajectory(timeinterval,position,orientation,velocity,acceleration);
            
            else
                %setting up TimeInterval for trajectory
                numberTimeSteps = length(obstacles_XML(i).trajectory.nodes);
                ts = obstacles_XML(i).trajectory.nodes(1).time;
                dt = obstacles_XML(i).trajectory.nodes(2).time - obstacles_XML(i).trajectory.nodes(1).time;
                tf = obstacles_XML(i).trajectory.nodes(numberTimeSteps).time;

                timeinterval = globalPck.TimeInterval(ts, dt, tf);

                %setting up Trajectory

                position = [[obstacles_XML(i).trajectory.nodes.x];[obstacles_XML(i).trajectory.nodes.y]];
                orientation  = [obstacles_XML(i).trajectory.nodes.orientation];
                velocity = [obstacles_XML(i).trajectory.nodes.velocity];
                acceleration = [obstacles_XML(i).trajectory.nodes.acceleration];

                
                trajectory  = globalPck.Trajectory(timeinterval,position,orientation,velocity,acceleration);
            end

            
            switch obstacles_XML(i).type %differentiating between vehicles and other dynamic obstacles
                %new vehicle, add here more vehicle types if needed
                case {'passengerCar','truck'}
                        args = {timeinterval_empty,[],[],obstacles_XML(i).shape.width, obstacles_XML(i).shape.length, ...
                            lanes.findLaneByPosition(position(:,1))};
                        vehicles(vehicle_count) = world.Vehicle(args{:});
                        vehicles(vehicle_count).setTrajectory(trajectory);
                        
                        vehicle_count=vehicle_count+1;
                    
                    
                %new dynamic obstacle
                otherwise 
                    args = {timeinterval_empty,[],[],obstacles_XML(i).shape.width, obstacles_XML(i).shape.length, ...
                        lanelets.findLaneletByPosition(position(:,1))};
                    obstacles_dynamic(obstacle_dynamic_count) = world.DynamicObstacle(args{:});
                    obstacles_dynamic(obstacle_dynamic_count).setTrajectory(trajectory);
                    
                    obstacle_dynamic_count=obstacle_dynamic_count+1;
            end
            
            
        case 'static'
            %new static obstacle
            
            if(length(obstacles_XML(i).trajectory.nodes)==1)
                args = {[obstacles_XML(i).trajectory.nodes(1).x;obstacles_XML(i).trajectory.nodes(1).y],...
                    obstacles_XML(i).trajectory.nodes(1).orientation,...
                    obstacles_XML(i).shape.width, obstacles_XML(i).shape.length, ...
                    lanes.findLaneByPosition([obstacles_XML(i).trajectory.nodes(1).x;obstacles_XML(i).trajectory.nodes(1).y])};
                obstacles_static(obstacle_static_count) = world.StaticObstacle(args{:});
                obstacle_static_count=obstacle_static_count+1;
            elseif (length(fields(obstacles_XML(i).trajectory))==3)
                warning('Static Obstacle with more than one position. Just the first is used.');
                args = {[obstacles_XML(i).trajectory.nodes(1).x;obstacles_XML(i).trajectory.nodes(1).y],...
                    obstacles_XML(i).trajectory.orientation,...
                    obstacles_XML(i).shape.width, obstacles_XML(i).shape.length, ...
                    lanes.findLaneByPosition([obstacles_XML(i).trajectory.nodes(1).x;obstacles_XML(i).trajectory.nodes(1).y])};
                obstacles_static(obstacle_static_count) = world.StaticObstacle(args{:});
                obstacle_static_count=obstacle_static_count+1;
            else
                warning('Couldnt detect which type of trajectory');
            end
        
        otherwise 
            warning('Couldnt detect whether static or dynamic');
            
    end
    
end





%------------- END CODE --------------