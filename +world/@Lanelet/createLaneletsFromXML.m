function [lanelets] = createLaneletsFromXML(lanelets_XML, adjacencyGraph_XML)
% createLaneletsFromXML - creates all Lanelet objects from the XML input
%
% Syntax:
%   [lanelets] = createLaneletsFromXML(lanelets_XML, adjacencyGraph_XML)
%
% Inputs:
%   lanelets_XML - structure of lanelets
%           .id
%           .left
%           .right
%           .speedLimit
%           .predecessor
%           .successor
%           .adjacentLeft
%                       .lanelet
%                       .driving
%           .adjacentRight
%                       .lanelet
%                       .driving
%   adjacencyGraph_XML - boolean whether adjacency is provided
%
% Outputs:
%   lanelets - array of Lanelet objects
%
% Other m-files required: createAdjacencyGraph()
% Subfunctions: none
% MAT-files required: none

% Author:       Markus Koschi
% Written:      02-Dezember-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

% construct lanelet objects
numLanelets = length(lanelets_XML);
lanelets(1,numLanelets) = world.Lanelet();
for i = 1:numLanelets
    if isfield(lanelets_XML(i), 'speedLimit') && ~isempty(lanelets_XML(i).speedLimit)
        args = {lanelets_XML(i).id, ...
            [lanelets_XML(i).left.nodes.x; lanelets_XML(i).left.nodes.y], ...
            [lanelets_XML(i).right.nodes.x; lanelets_XML(i).right.nodes.y], ...
            lanelets_XML(i).speedLimit};
    else
        args = {lanelets_XML(i).id, ...
            [lanelets_XML(i).left.nodes.x; lanelets_XML(i).left.nodes.y], ...
            [lanelets_XML(i).right.nodes.x; lanelets_XML(i).right.nodes.y]};
    end
    % create lanelet object
    lanelets(i) = world.Lanelet(args{:});
end

% add adjacent lanelets
if adjacencyGraph_XML
    for i = 1:numLanelets
        % add predecessors
        if isfield(lanelets_XML(i), 'predecessor') && ~isempty(lanelets_XML(i).predecessor)
            for j = 1:length(lanelets_XML(i).predecessor)
                index = [lanelets.id]' == lanelets_XML(i).predecessor(j).id;
                lanelets(i).addAdjacentLanelet('predecessor', lanelets(index));
            end
        end
        
        % add successors
        if isfield(lanelets_XML(i), 'successor') && ~isempty(lanelets_XML(i).successor)
            for j = 1:length(lanelets_XML(i).successor)
                index = [lanelets.id]' == lanelets_XML(i).successor(j).id;
                lanelets(i).addAdjacentLanelet('successor', lanelets(index));
            end
        end
        
        % add adjacent left
        if isfield(lanelets_XML(i), 'adjacentLeft') && ~isempty(lanelets_XML(i).adjacentLeft)
            for j = 1:length(lanelets_XML(i).adjacentLeft)
                index = [lanelets.id]' == lanelets_XML(i).adjacentLeft(j).lanelet.id;
                lanelets(i).addAdjacentLanelet('adjacentLeft', {lanelets(index), lanelets_XML(i).adjacentLeft(j).driving});
            end
        end
        
        % add adjacent right
        if isfield(lanelets_XML(i), 'adjacentRight') && ~isempty(lanelets_XML(i).adjacentRight)
            for j = 1:length(lanelets_XML(i).adjacentRight)
                index = [lanelets.id]' == lanelets_XML(i).adjacentRight(j).lanelet.id;
                lanelets(i).addAdjacentLanelet('adjacentRight', {lanelets(index), lanelets_XML(i).adjacentRight(j).driving});
            end
        end
    end
else
    % no adjacency graph exists, so create it
    world.Lanelet.createAdjacencyGraph(lanelets);
end

end

%------------- END CODE --------------