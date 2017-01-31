function [flipedLane] = flipLane(lane)
% flipLane - flip a lane including all its properties 
%
% Syntax:
%   lane = flipLane(obj,lane)
%
% Inputs:
%   obj - Lane object
%   lane - Lanes
%
% Output:
%   lane - returned fliped lane
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Rajat Koner
% Written:      06-January-2017
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

 %flipedLane=world.Lane.empty();
 
    flipedLane.leftBorder.vertices = fliplr(lane.leftBorder.vertices);
    flipedLane.leftBorder.distances = flip(lane.leftBorder.distances);
    flipedLane.leftBorder.curvatures = flip(lane.leftBorder.curvatures);
    
    flipedLane.rightBorder.vertices = fliplr(lane.rightBorder.vertices);
    flipedLane.rightBorder.distances = flip(lane.rightBorder.distances);
    flipedLane.rightBorder.curvatures = flip(lane.rightBorder.curvatures);
    
    flipedLane.center.vettices = fliplr(lane.center.vertices);    
    flipedLane.speedLimit = flip(lane.speedLimit);
    flipedLane.lanelets = lane.lanelets;
    %also flipped shortest path 
    flipedLane.shortestPath.xi = flip(lane.shortestPath.xi);
    flipedLane.shortestPath.indexBorder = flip(lane.shortestPath.indexBorder);
    flipedLane.shortestPath.side = flip(lane.shortestPath.side); 
    
   end

%------------- END CODE --------------