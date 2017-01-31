function [distance] = calcProjectedDistance(i, bound, position)
% calcProjectedDistance - project the position on the bound such that it is
% perpendicular to the pseudo tanget and calculate the distance of the
% projection point to vertice i
%
% Syntax:
%   [distance] = calcProjectedDistance(i, bound, position)
%
% Inputs:
%   i - vertice index of the bound which is closest to the object's position
%   bound - lane border
%   position - coordinates of the object
%
% Outputs:
%   distance - L2 norm from pLamda to vi
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: Bender et al., 2014, Lanelets: Efficient Map Representation for
% Autonomous Driving, III. Lanelts, D. Calculation and measures
%
% References in this function refer to this paper.

% Author:       Markus Koschi
% Written:      20-Oktober-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

% vertice_i of inner bound
vi = bound.vertices(:,i);

% find the previous and next vertices on the lane bound:
% (note that the bound of these vertices might not be the inner one)

% set v(i-2) and v(i-1)
if (i-2) > 0 % far away from lane beginning in forward driving direction
    % consider the segment v(i-1) -> v(i)
    flag = 1;
    viminus2 = bound.vertices(:,i-2);
    viminus1 = bound.vertices(:,i-1);
elseif (i-2) == 0
    % consider the segment v(i-1) -> v(i), in which v(i-1) is the first
    % vertice of the lane
    flag = 1;
    viminus2 = bound.vertices(:,i-1);
    viminus1 = viminus2;
else % iBorder == 1, i.e.
    % object is right at the beginning of the lane
    flag = 0; % consider the next segment v(i) -> v(i+1)
    viminus1 = vi;
end

% set v(i+1) and v(i+2)
if (i+2) <= length(bound.vertices)
    viplus2 = bound.vertices(:,i+2);
    viplus1 = bound.vertices(:,i+1);
elseif (i+1) == length(bound.vertices)
    viplus2 = bound.vertices(:,i+1);
    viplus1 = viplus2;
elseif i == length(bound.vertices)
    viplus2 = bound.vertices(:,i);
    viplus1 = viplus2;
    if ~flag % iBorder == 1 && iBorder == length(innerBound.vertices)
        error(['Lane %i is too short to properly calculate the projection of'...
            ' the object on the lane border'], bound.id(1));
    end
end

% construct the points (p) and tangents (t) of the base and the tip of the
% segment
ti = viplus1 - viminus1;
if flag % v(i-1) -> v(i)
    pBase = viminus1;
    tBase = vi - viminus2; % timinus1
    pTip = vi;
    tTip = ti;
else % ~flag: v(i) -> v(i+1)
    pBase = vi;
    tBase = ti;
    pTip = viplus1;
    tTip = viplus2 - vi; % tiplus1
end

% transform the coordinate system:
% translate by -pBase, such that pBase == 0
% rotate by theta, such that pTip(2) == 0
% (note that atan() does not work for all cases)
theta = atan2( -(pTip(2) - pBase(2)), (pTip(1) - pBase(1)) );

% points p:
% pBase = [0; 0]
% pTip = [l; 0]
l = norm(pTip - pBase);
% pTip_x = l = (pTip(1) - pBase(1))*cos(theta) - (pTip(2) - pBase(2))*sin(theta)
% pTip_y = 0 = (pTip(1) - pBase(1))*sin(theta) + (pTip(2) - pBase(2))*cos(theta)

% transform the tangent vectors into new coordinate system,
% i.e. rotate with theta
tBase_Rot(1) = tBase(1) * cos(theta) -  tBase(2) * sin(theta);
tBase_Rot(2) = tBase(1) * sin(theta) +  tBase(2) * cos(theta);
tTip_Rot(1) = tTip(1) * cos(theta) -  tTip(2) * sin(theta);
tTip_Rot(2) = tTip(1) * sin(theta) +  tTip(2) * cos(theta);

% transform the tangents such that t = [1; m], i.e. slopes m = ty / tx
% tBase = [1; mBase]
mBase = tBase_Rot(2) / tBase_Rot(1);
% tTip = [1; mTip]
mTip = tTip_Rot(2) / tTip_Rot(1);

% transform the position of the object = [x, y] in new coordinate system:
% translate
x_Trans = position(1) - pBase(1);
y_Trans = position(2) - pBase(2);
% rotate
x = x_Trans * cos(theta) -  y_Trans * sin(theta);
y = x_Trans * sin(theta) +  y_Trans * cos(theta);

% solve equations (1) - (4) for parameter lamda (equation (5))
lamda = (x + y * mBase) / (l - y * (mTip - mBase));

% distance from pLamda to vi (distance is equal in both coordinate systems)
if flag % (vi == pTip)
    % i.e. distance from pLamda to pTip
    distance = (1 - lamda) * l; %previous verified version
else % (vi == pBase)
    % i.e. distance from pLamda to pBase
    distance = - lamda * l;
end

% % figure() % in transformed coordinates (X_)
% % plot(0,0,'k.') %pBase
% % hold on
% % plot(l,0,'k.') %pTip
% % plot(x,y,'r.') %x
% % plot(lamda*l, 0, 'rx') %pLamda
% %
% % alpha = 0:0.1:1;
% % pBase_ = [0; 0];
% % pTip_ = [l; 0];
% % tBase_ = [1; mBase];
% % tTip_ = [1; mTip];
% % plot(pBase_(1)+alpha*tBase_(1), pBase_(2)+alpha*tBase_(2), 'g-')
% % plot(pTip_(1)+alpha*tTip_(1), pTip_(2)+alpha*tTip_(2), 'g--')

%------------- END CODE --------------

end