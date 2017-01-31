function computeOccupancyForObstacle(obj, map, timeInterval)
% computeOccupancyForObstacle - frame for occupancy predicition of all obstacles
%
% Syntax:
%   
%
% Inputs:
%   
%
% Outputs:
%   
%
% Other m-files required: computeOccupancyCore
% Subfunctions: none
% MAT-files required: none
%
% See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic
% Participants on Arbitrary Road Networks, IV. Occupancy Prediction

% Author:       Markus Koschi
% Written:      28-October-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

% set inLaneId if isempty(obj.inLaneId) or update inLaneId
% TODO

% compute the occupancy
%tic

obj.occupancy = obj.occupancy.computeOccupancyCore(obj, map, timeInterval);

%fprintf('Computation time for the occupancy of obstacle %d: %f seconds.\n', obj.id, toc)

end

%------------- END CODE --------------