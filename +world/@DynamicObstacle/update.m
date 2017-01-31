function update( varargin )
% update - updates the properties of a DynamicObstacle object by the values
% of the trajectory at the specified time
%
% Syntax:
%   update(obj, t)
%
% Inputs:
%   obj - DynamicObstacle object
%   t - current time of the object
%
% Outputs:
%   none
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Stefanie Manzinger, Markus Koschi
% Written:      29-Nomvember-2016
% Last update:  31-January-2017
%
% Last revision:---

%------------- BEGIN CODE --------------

    if nargin == 2
        obj = varargin{1};
        t = varargin{2};
    end
    
    % check if t is in the time interval of the object
    if obj.trajectory.timeInterval.getIndex(t) > 0 && ...
            obj.trajectory.timeInterval.getIndex(t) <= size(obj.trajectory.position,2)
        
        % update all properties
        obj.position = obj.trajectory.getPosition(t);
        obj.orientation = obj.trajectory.getOrientation(t);
        obj.velocity = obj.trajectory.getVelocity(t);
        obj.acceleration = obj.trajectory.getAcceleration(t);
        obj.time = t;
        % ToDo: update lanelet
    end
end

%------------- END CODE --------------