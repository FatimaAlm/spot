function [bound, iBorder_Bound] = constructBound(iPath_Obstacle, xiBound, typeOfBound, xi, shortestPath, leftBorder, rightBorder)
%constructBound - construct the bound of the inflection point segmentation
% along the shortest path, such that the reach xiBound is enclosed
%
% Syntax:
%   [bound, iBorder_Bound] = constructBound(iPath_Obstacle, xiBound, typeOfBound, xi, shortestPath, leftBorder, rightBorder)
%
% Inputs:
%   iPath_Obstacle - index in shortest path of the vertice which is closest to the obstacle
%   xiBound - closest or furthest longitudinal reach of the obstacle
%   typeOfBound - string: 'front' or 'final'
%   shortestPath - shortest path through the lane
%   leftBorder - left border of the lane
%   rightBorder - right border of the lane
%
% Outputs:
%   bound - point coordinates of the bound
%   iBorder_Bound - index of the lane border at which the segmentation
%                   starts or ends, respectively
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic
% Participants on Arbitrary Road Networks, IV. Occupancy Prediction, B.
% Lane-Following Occupancy (Abstraction M_2), Definition 8
% (Inflection-point segmentation)
%
% References in this function refer to this paper.

% Author:       Markus Koschi
% Written:      23-November-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

% find the bound of the occupancy, such that xiBound is enclosed
% (over-approximation):
% (note that xiBound might be negative if backward driving is allowed)
if xiBound >= xi(iPath_Obstacle)
    % follow the shortest path forwards until just before xiClosest is reached
    for j = iPath_Obstacle:(length(xi)-1)
        if xi(j+1) > xiBound
            % the bound is between the vertex j and the next vertex (j+1)
            break;
        end
        % else step forward to vertice (j+1)
    end
else % (xiBound - xiObstacle) < 0
    % check if iPath_Obstacle is not at the beginning of the bound
    if iPath_Obstacle > 1
        % follow the shortest path backwards until xiBound is passed
        for j = (iPath_Obstacle-1):-1:1
            if xi(j) < xiBound
                % the bound is between vertex j and previous vertex (j+1),
                % which is closer to the obstacle
                break;
            end
            % else step back to vertex (j-1)
        end
    else % iPath_Obstacle == 1
        j = 1;
    end
end

% construct the bound between the vertices j and (j+1):
% set indexes on shortest path and borders
iPath_Bound_j = j;
iPath_Bound_jplus1 = j+1;
iBorder_Bound_j = shortestPath.indexBorder(iPath_Bound_j);
iBorder_Bound_jplus1 = shortestPath.indexBorder(iPath_Bound_jplus1);

% vector along the left and right border
% (vector equation: pointsBound = vertex(Bound_j) + lamda * vector)
vectorBoundLeft = leftBorder.vertices(:,iBorder_Bound_jplus1) - leftBorder.vertices(:,iBorder_Bound_j);
vectorBoundRight = rightBorder.vertices(:,iBorder_Bound_jplus1) - rightBorder.vertices(:,iBorder_Bound_j);

% distance on inner bound (of lane) between vertex j and xiBound
distanceBound = xiBound - xi(iPath_Bound_j);

% parameter lamda, such that xiBound is reached by vector
lamdaBoundLeft = distanceBound / norm(vectorBoundLeft);
lamdaBoundRight = distanceBound / norm(vectorBoundRight);

% calculate the points of the bound, which are on the left and right border,
% by the vector equation
pBoundLeft = leftBorder.vertices(:,iBorder_Bound_j) + lamdaBoundLeft * vectorBoundLeft;
pBoundRight = rightBorder.vertices(:,iBorder_Bound_j) + lamdaBoundRight * vectorBoundRight;

% the bound runs from the right to the left border at pBound

% save the the points of the bound and
% the index of the lane border at which the occupancy will start or end
switch typeOfBound
    case 'front'
        % the front bound is from the right to the left border at pBound
        bound = [pBoundRight, pBoundLeft];
        % and continues at index (j+1) on the border
        iBorder_Bound = iBorder_Bound_jplus1;
    case 'final'
        % the final bound is from the left to the right border at pBound
        bound = [pBoundLeft, pBoundRight];
        % and runs until index j on the border
        iBorder_Bound = iBorder_Bound_j;
end

% DEGUB:
% check the actual xi of the bound, whether the bound over-approximates the
% reach xiBound
if sign(distanceBound) == 1
    xiBoundLeft = norm(pBoundLeft - leftBorder.vertices(:,iBorder_Bound_j)) + xi(iPath_Bound_j);
    xiBoundRight = norm(pBoundRight - rightBorder.vertices(:,iBorder_Bound_j)) + xi(iPath_Bound_j);
else % sign(distanceFrontBound) == -1
    xiBoundLeft = - norm(pBoundLeft - leftBorder.vertices(:,iBorder_Bound_j)) + xi(iPath_Bound_j);
    xiBoundRight = - norm(pBoundRight - rightBorder.vertices(:,iBorder_Bound_j)) + xi(iPath_Bound_j);
end
switch typeOfBound
    case 'front'
        % the computed front bound must not be further away than the
        % closest reach
        if (xiBoundLeft - xiBound) > 10e-9 && (xiBoundRight - xiBound) > 10e-9
            warning('Front bound is further away than xiClosest, i.e. no over-approximation.')
        end
    case 'final'
        % the computed final bound must not be closer than the furthest
        % reach
        if (xiBoundLeft - xiBound) < -10e-9  && (xiBoundRight - xiBound) < -10e-9
            warning('Final bound is closer than xiFurthest, i.e. no over-approximation.')
        end
end

% % % DEBUG: plot
% % alpha = 0:0.5:2;
% % plot(rightBorder.vertices(1,iBorder_Bound_j), rightBorder.vertices(2,iBorder_Bound_j), 'rx')
% % plot(rightBorder.vertices(1,iBorder_Bound_jplus1), rightBorder.vertices(2,iBorder_Bound_jplus1), 'r*')
% % plot(rightBorder.vertices(1,iBorder_Bound_j) + alpha * vectorBoundRight(1,:), rightBorder.vertices(2,iBorder_Bound_j) + alpha * vectorBoundRight(2,:), 'r.')
% % plot(leftBorder.vertices(1,iBorder_Bound_j), leftBorder.vertices(2,iBorder_Bound_j), 'rx')
% % plot(leftBorder.vertices(1,iBorder_Bound_jplus1), leftBorder.vertices(2,iBorder_Bound_jplus1), 'r*')
% % plot(leftBorder.vertices(1,iBorder_Bound_j) + alpha * vectorBoundLeft(1,:), leftBorder.vertices(2,iBorder_Bound_j) + alpha * vectorBoundLeft(2,:), 'r.')
% % plot(bound(1,:), bound(2,:), 'r-')

end

%------------- END CODE --------------