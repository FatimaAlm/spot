classdef Lanelet < handle
    % Lanelet - repesents single piece of a road
    %
    % Syntax:
    %  object constructor: obj = Lanelet(varargin)
    %
    % Inputs:
    %   id - a unique id
    %   leftBorderVertices - vertices of left border
    %   rightBorderVertices - vertices of right border
    %   speedLimit - official speed limit of the Lanelet
    %   predecessorLanelets - previous Lanelet (multiple for a road fork)
    %   successorLanelets - longitudinally adjacent Lanlet (multiple for a road fork)
    %   adjacentLeft - left adjacent lanelet with driving tag
    %   adjacentRight - right adjacent lanelet with driving tag
    %
    % Outputs:
    %   obj - generated object
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    %
    % See also: Bender et al., 2014, Lanelets: Efficient Map Representation
    % for Autonomous Driving
    
    % Author:       Markus Koschi
    % Written:      02-Dezember-2016
    % Last update:
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (SetAccess = protected, GetAccess = public)
        id = []; % a unique id
        leftBorderVertices = []; % vertices of left border
        rightBorderVertices = []; % vertices of right border
        speedLimit = inf; % official speed limit of the Lanelet
        predecessorLanelets  = world.Lanelet.empty();  % previous Lanelet (multiple for a road fork)
        successorLanelets = world.Lanelet.empty(); % longitudinally adjacent Lanlet (multiple for a road fork)
        adjacentLeft = []; % left adjacent lanelet with driving tag
        adjacentRight = []; % right adjacent lanelet with driving tag
        centerVertices = []; % center vertives of a lane
    end
    
    methods
        % class constructor
        function obj = Lanelet(id, leftBorderVertices, rightBorderVertices, ...
                speedLimit, predecessorLanelets, successorLanelets, adjacentLeft, adjacentRight)
           persistent laneletIdCount;
           if isempty(laneletIdCount)
                laneletIdCount = 0;
           end
            
            if nargin >= 3
                % set id
                if isempty(id) || (abs(id) < laneletIdCount)
                    laneletIdCount = laneletIdCount + 1;
                    obj.id = laneletIdCount;
                else
                    obj.id = id;
                end
                
                %TODO: revise
                % if the vertices of the left border are shared with a laterally
                % adjacent lanelet which has an opposite driving direction,
                % flip the vertices of the left border
                if norm(rightBorderVertices(:,1) - leftBorderVertices(:,1)) > ...
                        (norm(rightBorderVertices(:,1) - leftBorderVertices(:,end)) + ...
                        norm(rightBorderVertices(:,round(size(rightBorderVertices,2)/2)) - leftBorderVertices(:,round(size(leftBorderVertices,2)/2))))
                    leftBorderVertices = fliplr(leftBorderVertices);
                end
                % set border vertices
                %obj.leftBorderVertices = leftBorderVertices;
                %obj.rightBorderVertices = rightBorderVertices;
                %obj.alignBorderVertices();
                [obj.leftBorderVertices, obj.rightBorderVertices] = obj.alignBorderPoints(leftBorderVertices, rightBorderVertices);
                
                % set center vertices by calculating them from the border
                % vertices
                obj.createCenterVertices();
            end
            
            % set speed limit
            if nargin >= 4 && ~isempty(speedLimit)
                obj.speedLimit = speedLimit;
            end
            
            % set adjacent lanelets
            if nargin == 8
                if ~isempty(predecessorLanelets)
                    obj.predecessorLanelets = predecessorLanelets;
                end
                if ~isempty(successorLanelets)
                    obj.successorLanelets = successorLanelets;
                end
                if ~isempty(adjacentLeft)
                    obj.adjacentLeft = adjacentLeft;
                end
                if ~isempty(adjacentRight)
                    obj.adjacentRight = adjacentRight;
                end
            end
        end
        
        % methods in seperate files
        [lBorder, rBorder] = alignBorderPoints(obj, leftBorder, rightBorder)
        [pt_out, dist, idx] = closestPointOnLaneBorder(obj, point, rBorder)
        createCenterVertices(obj)
        [successors] = findAllSuccessorLanelets(obj, successors)
                
        % visualization methods in seperate files
        disp(obj)
        plot(obj)
    end
    
    % static methods in seperate files
    methods(Static)
        % create lanelets from xml data
        lanelets = createLaneletsFromXML(lanelets_XML, adjacencyGraph_XML)
        
        % create the adjacency graph
        createAdjacencyGraph(lanelets)
        %check adjacency among two lanelets
        [notParallel,distanceLK,distanceRK] = checkLaneletAdjacency(l_k,r_k,j1,j2,length,widthK,widthJ)
        %temporary check adjacency among two lanelets
        [notParallel,distanceLK,distanceRK] = tempCheckLaneletAdjacency(l_k,r_k,j1,j2,length,widthK,widthJ)

    end
end

%------------- END CODE --------------