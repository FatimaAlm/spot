function [curvature] = calcCurvature(x, y)
% calcCurvature - calculate the well-known signed curvature of the polyline
%
% Syntax:
%   [curvature] = calcCurvature(x, y)
%
% Inputs:
%   x - x-coordinates of the polyline
%   y - y-coordinates of the polyline
%
% Outputs:
%   curvature - curvature of the given polyline
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: ---

% Author:       Markus Koschi
% Written:      05-Oktober-2016
% Last update:  19-Oktober-2016 (signOfCurv --> curvature)
%
% Last revision:---

%------------- BEGIN CODE --------------

% calculate the gradients
dx = gradient(x);
ddx = gradient(dx);
dy = gradient(y);
ddy = gradient(dy);

% calculate the signed curvature
curvature = (dx .* ddy - ddx .* dy) ./ ((dx.^2 + dy.^2).^(3/2));

end

%------------- END CODE --------------