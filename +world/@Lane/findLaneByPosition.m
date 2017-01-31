function [lanes] = findLaneByPosition(obj, position)
% findLaneByPosition - find the lane(s) by the ostacle's position
%
% Syntax:
%   lanes = findLaneByPosition(obj, position)
%
% Inputs:
%   obj - Lane object(s)
%   position - position of an obstacle
%
% Outputs:
%   lanes - Lane object(s)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Rajat Koner
% Written:     23-December-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

lanes = world.Lane.empty();

for i = 1:numel(obj)
    xv = [obj(i).leftBorder.vertices(1,:),fliplr(obj(i).rightBorder.vertices(1,:)),obj(i).leftBorder.vertices(1,1)];
    yv = [obj(i).leftBorder.vertices(2,:),fliplr(obj(i).rightBorder.vertices(2,:)),obj(1).leftBorder.vertices(2,1)];
    % call inpolygon for checking position in the lane
    [in,on] = inpolygon(position(1,:), position(2,:), xv, yv);
    if(in(1)==1)
        lanes(end+1) = obj(i);
    end
end
end

%------------- END CODE --------------