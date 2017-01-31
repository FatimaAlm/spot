function plot(varargin)
% plot - plots a Obstacle object
%
% Syntax:
%   plot(obj, timeInterval)
%
% Inputs:
%   obj - Obstacle object
%   timeInterval - TimeInterval object
%   flag_egoVehicle - for egoVehicle
%
% Outputs:
%   none
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Markus Koschi
% Written:      15-November-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

if nargin == 1
    % plot the obstacle for the current time
    obj = varargin{1};
    timeInterval = globalPck.TimeInterval.empty();
    flag_egoVehicle = false;
elseif nargin == 2
    % plot the obstacle for the time interval
    obj = varargin{1};
    timeInterval = varargin{2};
    flag_egoVehicle = false;
elseif nargin == 3
    % plot the obstacle for the time interval
    obj = varargin{1};
    timeInterval = varargin{2};
    flag_egoVehicle = varargin{3};
end

for i = 1:numel(obj)
    
    % plot obstacle's occupancy
    if ~isempty(obj(i).occupancy) && ~isempty(obj(i).occupancy(1,1).timeInterval)
        obj(i).occupancy.plot(timeInterval, obj(i).color);
    end
    
    % plot obstacle's center
    if globalPck.PlotProperties.SHOW_OBSTACLES_CENTER
        plot(obj(i).position(1), obj(i).position(2), 'Color', obj(i).color, 'Marker', 'x');
    end
    
    % plot obstacle's dimensions
    if ~isempty(obj(i).position) && ~isempty(obj(i).orientation)
        vertices = geometry.rotateAndTranslateVertices(geometry.addObjectDimensions([0;0], obj(i).length, obj(i).width), obj(i).position, obj(i).orientation);
        %patch(vertices(1,:), vertices(2,:), 1, 'FaceColor', obj.color, 'FaceAlpha', 1, 'EdgeColor', obj.color);
        fill(vertices(1,:), vertices(2,:), obj(i).color, 'FaceAlpha', 1, 'EdgeColor', 'k');
        %fill(vertices(1,:), vertices(2,:), [0 0 1], 'FaceAlpha', 1, 'EdgeColor', 'k');
    end
    
    hold on
    
    % plot obstacle's trajectory
    if isa(obj(i), 'world.DynamicObstacle') && globalPck.PlotProperties.SHOW_TRAJECTORIES ...
            && ~isempty(obj(i).trajectory)
        obj(i).trajectory.plot(timeInterval, obj(i).color);
    end
    
    % print description
    if globalPck.PlotProperties.SHOW_OBJECT_NAMES && ~isempty(obj(i).position)
        if ~flag_egoVehicle
            string = sprintf('obstacle %d', obj(i).id);
        else
            string = sprintf('ego Vehicle %d', obj(i).id);
        end
        text(obj(i).position(1), obj(i).position(2), 0, ['\leftarrow ', string]);
    end
end

end

%------------- END CODE --------------