function [laneStruct] = combineLaneletAndSuccessors(laneStruct, currentLanelet, k)
% combineLaneletAndSuccessors - builds a struct with lane properties of the
% current lanelet and all longitudinally adjacent lanelets
%
% Syntax:
%   [laneStruct] = combineLaneletAndSuccessors(laneStruct, currentLanelet, k)
%
% Inputs:
%   laneStruct - struct with lane properties
%   currentLanelet - current lanelet in the search along the lane
%   j - index of lane struct
%
% Outputs:
%   laneStruct - struct with lane properties
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Markus Koschi
% Written:      05-Dezember-2016
% Last update:  13-January-2017
%
% Last revision:---

%------------- BEGIN CODE --------------

if ~isempty(currentLanelet)
    %     % old code
    %     % if the current lanelet has different laterally adjecent lanelets than
    %     % the backward longitudinally adjacent lanelet (i.e. lanelets(end)),
    %     % a new lane must possibly be created (see next if conditions)
    %     % (note if both lanelets have no laterally adjecent lanelet, it is
    %     % considered equal)
    %     if (~isequal(laneStruct(k).lanelets(end).adjacentLeft, currentLanelet.adjacentLeft) || ...
    %             ~isequal(laneStruct(k).lanelets(end).adjacentRight, currentLanelet.adjacentRight))
    %         flag_merge = false;
    %
    %         % check whether the two lanelets merge eventually. if they merge, a
    %         % new lane must not be created
    %         % (find all successor lanelets of the laterally adjacent lanelet
    %         % and all successors lanelets of the current lanelet, and check if
    %         % any of them is the same)
    %         if ~isempty(laneStruct(k).lanelets(end).adjacentLeft)
    %             adjacentRightSuccessors = laneStruct(k).lanelets(end).adjacentLeft.lanelet.findAllSuccessorLanelets(world.Lanelet.empty());
    %             currentLaneletSuccessors = currentLanelet.findAllSuccessorLanelets(world.Lanelet.empty());
    %             flag_merge = any(ismember(adjacentRightSuccessors, currentLaneletSuccessors));
    %         end
    %         if ~isempty(laneStruct(k).lanelets(end).adjacentRight)
    %             adjacentRightSuccessors = laneStruct(k).lanelets(end).adjacentRight.lanelet.findAllSuccessorLanelets(world.Lanelet.empty());
    %             currentLaneletSuccessors = currentLanelet.findAllSuccessorLanelets(world.Lanelet.empty());
    %             flag_merge = any(ismember(adjacentRightSuccessors, currentLaneletSuccessors));
    %         end
    %
    %         if ~flag_merge
    %             % as the two lanelets never merge, a seperate new lane must be
    %             % created to fullfil the lane definition (two lanes are only
    %             % laterally adjacent if adjacenct along their whole length)
    %
    %             % one lane ends here and a new one is created to continue with the current lanelet
    %             laneStruct(k+1) = laneStruct(k);
    %             k = k + 1;
    %         end
    %     end
    
    % add the lane properties of the current lanelet to the struct
    laneStruct(k).leftBorderVertices = [laneStruct(k).leftBorderVertices, currentLanelet.leftBorderVertices(:,2:end)];
    laneStruct(k).rightBorderVertices = [laneStruct(k).rightBorderVertices, currentLanelet.rightBorderVertices(:,2:end)];
    laneStruct(k).lanelets(end+1) = currentLanelet;
    % choose the higher speed limit for the connection point
    % (over-approximation)
    laneStruct(k).speedLimit = [laneStruct(k).speedLimit(1:end-1), ...
        max(laneStruct(k).speedLimit(end),currentLanelet.speedLimit), ...
        currentLanelet.speedLimit * ones(1,size(currentLanelet.leftBorderVertices,2)-1)];
    laneStruct(k).centerVertices = [laneStruct(k).centerVertices, currentLanelet.centerVertices(:,2:end)];
    
    % for all successor lanelets, copy this lane in the next rows of the struct
    laneStruct((k+1):(k+numel(currentLanelet.successorLanelets)-1)) = laneStruct(k);
    
    % continue for all succesor lanelets
    for n = 1:numel(currentLanelet.successorLanelets)
        if ~isequal(laneStruct(k+n-1).lanelets(end), laneStruct(k).lanelets(length(laneStruct(k+n-1).lanelets)))
            % the recursive function combineLaneletAndSuccessors has
            % overwritten the current lanelet (at row k+n-1) due to a road fork
            % hence, create new row o with laneStruct(k).lanelets(1:currentLanelet)
            o = size(laneStruct,2)+1;
            laneStruct(o).lanelets = laneStruct(k).lanelets(1);
            laneStruct(o).leftBorderVertices = laneStruct(k).lanelets(1).leftBorderVertices;
            laneStruct(o).rightBorderVertices = laneStruct(k).lanelets(1).rightBorderVertices;
            laneStruct(o).speedLimit = laneStruct(k).lanelets(1).speedLimit * ones(1,size(laneStruct(k).lanelets(1).leftBorderVertices,2));
            laneStruct(o).centerVertices = laneStruct(k).lanelets(1).centerVertices;            
            p = 2;
            while ~isequal(laneStruct(o).lanelets(end), currentLanelet)
                laneStruct(o).lanelets(end+1) = laneStruct(k).lanelets(p);
                laneStruct(o).leftBorderVertices = [laneStruct(o).leftBorderVertices, laneStruct(k).lanelets(p).leftBorderVertices(:,2:end)];
                laneStruct(o).rightBorderVertices = [laneStruct(o).rightBorderVertices, laneStruct(k).lanelets(p).rightBorderVertices(:,2:end)];
                laneStruct(o).speedLimit = [laneStruct(o).speedLimit(1:end-1), ...
                    max(laneStruct(o).speedLimit(end),laneStruct(k).lanelets(p).speedLimit), ...
                    laneStruct(k).lanelets(p).speedLimit * ones(1,size(laneStruct(k).lanelets(p).leftBorderVertices,2)-1)];
                laneStruct(o).centerVertices = [laneStruct(o).centerVertices, laneStruct(k).lanelets(p).centerVertices(:,2:end)];
                p = p + 1;
            end
            % continue with row o
            laneStruct = world.Lane.combineLaneletAndSuccessors(laneStruct, currentLanelet.successorLanelets(n), o);
        else
            % continue with row k+n-1
            laneStruct = world.Lane.combineLaneletAndSuccessors(laneStruct, currentLanelet.successorLanelets(n), k+n-1);
        end
    end
end

end

%------------- END CODE --------------