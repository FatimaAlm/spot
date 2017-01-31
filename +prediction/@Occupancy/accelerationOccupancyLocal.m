function [q] = accelerationOccupancyLocal(vx, a_max, tk, tkplus1, constraint3)
% accelerationOccupancyLocal - compute the occupancy ploygon for one time step in
% local coordinates (acceleration-based occupancy; abstraction M_1)
%
% Syntax:
%   [polygon] = accelerationOccupancyLocal(vx, a_max, t_k, t_kplus1)
%
% Inputs:
%   vx - velocity of the car in local coordinates
%   a_max - maximum acceleration of the car
%   tk - start time of the time step
%   tkplus1 - final time of the time step
%   constraint3 - boolean, whether driving backwards is not allowed
%
% Outputs:
%   q - polygon with six vertices representing the over-approximated hull
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic
% Participants on Arbitrary Road Networks, IV. Occupancy Prediction, A.
% Acceleration-Based Occupancy (Abstraction M_1)
%
% References in this function refer to this paper.

% Author:       Markus Koschi
% Written:      15-September-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

% set velocity vector in local coordinates, where vx = velocity, vy = 0
v = [vx; 0];

% compute center coordinates [cx, cy] (equation (3))
c_tk = v * tk;
c_tkplus1 = v * tkplus1;

% compute radius (equation (3))
r_tk = 1/2 * abs(a_max) * tk^2;
r_tkplus1 = 1/2 * abs(a_max) * tkplus1^2;

% calculate the boundary bx(tkplus1): (p. 7)
% time at d(bx(t))/dt = 0
tStar = sqrt(2/3) * (vx / abs(a_max));
if (tkplus1 < tStar)
    bx_tkplus1 = (vx * tkplus1) - (a_max^2 * tkplus1^3 / (2*vx));
else
    % bx_tkplus1 = bx_tstar
    bx_tkplus1 = (vx * tStar) - (a_max^2 * tStar^3 / (2*vx));
end

% previous version (TODO: remove)
% % compute vertices of the convex hull (Lemma 1)
% q1 = [c_tk(1,:) - r_tk; c_tk(2,:) + r_tk];
% q2 = [bx_tkplus1; c_tkplus1(2,:) + r_tkplus1];
% q3 = [c_tkplus1(1,:) + r_tkplus1; c_tkplus1(2,:) + r_tkplus1];
% q4 = [c_tkplus1(1,:) + r_tkplus1; c_tkplus1(2,:) - r_tkplus1];
% q5 = [bx_tkplus1; c_tkplus1(2,:) - r_tkplus1];
% q6 = [c_tk(1,:) - r_tk; c_tk(2,:) - r_tk];
%
% % polygon q
% q = [q1, q2, q3, q4, q5, q6];

% compute vertices of the convex hull (Lemma 1 with modification):
% (polygon to over-approximate the occupancy described by two circles at
% tk and tkplus1)
% vertices around the first circle (q1 and q6):
if constraint3
    % enforce constraint 3 (driving backwards is forbidden) (p. 7)
    if (tk < tStar) || (vx == 0)
        q1_x = c_tk(1,:) - r_tk;
    else
        % q1_x = bx_tStar, i.e. maximum value of bx
        q1_x = (vx * tStar) - (a_max^2 * tStar^3 / (2*vx));
    end
    q1 = [q1_x; c_tk(2,:) + r_tk];
    q6 = [q1_x; c_tk(2,:) - r_tk];
else % ~constraint3
    if ((c_tk(1,:) - r_tk) > (c_tkplus1(1,:) - r_tkplus1))
        % take the hull of the second circle at tkplus1, if the first circle
        % is in front of (within) the second one
        q1 = [c_tkplus1(1,:) - r_tkplus1; c_tkplus1(2,:) + r_tkplus1];
        q6 = [c_tkplus1(1,:) - r_tkplus1; c_tkplus1(2,:) - r_tkplus1];
    else
        % hull of the first circle
        q1 = [c_tk(1,:) - r_tk; c_tk(2,:) + r_tk];
        q6 = [c_tk(1,:) - r_tk; c_tk(2,:) - r_tk];
    end
end

% vertices around second circle (q2, q3, q4, and q5):
q2 = [bx_tkplus1; c_tkplus1(2,:) + r_tkplus1];
q3 = [c_tkplus1(1,:) + r_tkplus1; c_tkplus1(2,:) + r_tkplus1];
q4 = [c_tkplus1(1,:) + r_tkplus1; c_tkplus1(2,:) - r_tkplus1];
q5 = [bx_tkplus1; c_tkplus1(2,:) - r_tkplus1];

% set the polygon q
if ~isnan(q2(1)) && ~isnan(q5(1))
    q = [q1, q2, q3, q4, q5, q6];
else
    % q2 and q5 are not defined if vx==0
    q = [q1, q3, q4, q6];
end

% % test: plot circles and [bx, by]
% figure()
% geometry.plotCircle(c_tk(1,:),c_tk(2,:),r_tk);
% hold on
% geometry.plotCircle(c_tkplus1(1,:),c_tkplus1(2,:),r_tkplus1);
% bx_tk = (vx * tk) - (a_max^2 * tk^3 / (2*vx));
% by_tk = sqrt( (1/4 * a_max^2 * tk^4) - (a_max^2 * tk^3 / (2*vx))^2 );
% bx_tkplus1 = (vx * tkplus1) - (a_max^2 * tkplus1^3 / (2*vx));
% by_tkplus1 = sqrt( (1/4 * a_max^2 * tkplus1^4) - (a_max^2 * tkplus1^3 / (2*vx))^2 );
% bx_tstar = (vx * tStar) - (a_max^2 * tStar^3 / (2*vx));
% by_tstar = 0;
% plot(bx_tk,by_tk,'b*')
% plot(bx_tkplus1,by_tkplus1,'r*')
% plot(bx_tstar,by_tstar,'k*')
% axis equal
% patch(q(1,:),q(2,:),1, 'FaceColor', 'r', 'FaceAlpha', 0.2)
% %patch(qNew(1,:),qNew(2,:),1, 'FaceColor', 'b', 'FaceAlpha', 0.1)
% hold off

end

%------------- END CODE --------------