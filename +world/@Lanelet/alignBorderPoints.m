function [lBorder, rBorder] = alignBorderPoints(obj,leftBorder, rightBorder)
% alignBorderPoints - adds point to left/right border in order to
% return two borders with identical number of points that align
% perfectly (line through points is orthogonal to borders)
% %TODO: Rajat: function needs revision
%
% Syntax:
%   [lBorder, rBorder] = alignBorderPoints(obj, leftBorder, rightBorder)
%
% Inputs:
%   leftBorder  - left border
%   rightBorder - right border
%
% Outputs:
%   lBorder = new calculated left border
%   rBorder = new calculated right border
%
% Other m-files required: %TODO
% Subfunctions: closestPointOnLaneBorder()
% MAT-files required: none
%
% See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic Participants on
% Arbitrary Road Networks, III. A. Road Network Representation

% Author:       
% Written:      08-November-2016
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------

%define function insertInMatrix for inserting a vector int oa
%matrix
insertInMatrix = @(e,m,n)cat(2,m(:,1:n-1),e,m(:,n:end));


%mirror points from left to right and from right to left
rBorder = rightBorder;
lBorder = leftBorder;

%mirror every point exept the first and last points
rightBorder(:,1) = [];
rightBorder(:,end) = [];
leftBorder(:,1) = [];
leftBorder(:,end) = [];

%from left to right
for point = leftBorder
    %calculate point
    [pt_out, dist, idx] = obj.closestPointOnLaneBorder(point, rBorder);
    if ~isnan(pt_out(1)) && ~isnan(pt_out(2))
        %find correct index to insert point
        %get direction of lane
        if idx ~= 1
            if rBorder(1,idx-1) < rBorder(1,idx)
                rightDir = true;
            else
                rightDir = false;
            end
        else
            if rBorder(1,idx) < rBorder(1,idx+1)
                rightDir = true;
            else
                rightDir = false;
            end
        end
        %add point to rBorder
        if rightDir
            if pt_out(1) > rBorder(1,idx)
                rBorder = insertInMatrix(pt_out, rBorder, idx+1);
            else
                rBorder = insertInMatrix(pt_out, rBorder, idx);
            end
        else
            if pt_out(1) < rBorder(1,idx)
                rBorder = insertInMatrix(pt_out, rBorder, idx+1);
            else
                rBorder = insertInMatrix(pt_out, rBorder, idx);
            end
        end
    else
        %delete point from lBorder
        k = find(lBorder(1,:) == point(1));
        lBorder(:,k) = [];
    end
end

%from right to left
for point = rightBorder
    %calculate point
    [pt_out, dist, idx] = obj.closestPointOnLaneBorder(point, lBorder);
    if ~isnan(pt_out(1)) && ~isnan(pt_out(2))
        %find correct index to insert point
        %get direction of lane
        if idx ~= 1
            if lBorder(1,idx-1) < lBorder(1,idx)
                rightDir = true;
            else
                rightDir = false;
            end
        else
            if lBorder(1,idx) < lBorder(1,idx+1)
                rightDir = true;
            else
                rightDir = false;
            end
        end
        %add point to rBorder
        if rightDir
            if pt_out(1) > lBorder(1,idx)
                lBorder = insertInMatrix(pt_out, lBorder, idx+1);
            else
                lBorder = insertInMatrix(pt_out, lBorder, idx);
            end
        else
            if pt_out(1) < lBorder(1,idx)
                lBorder = insertInMatrix(pt_out, lBorder, idx+1);
            else
                lBorder = insertInMatrix(pt_out, lBorder, idx);
            end
        end
    else
        %delete element from rBorder
        k = find(rBorder(1,:) == point(1));
        rBorder(:,k) = [];
    end
end

%eliminate points that are too close
len = length(lBorder)-1;
k = 1;
while k < len
    if abs(lBorder(1,k+1) - lBorder(1,k)) < 0.01
        lBorder(:,k+1) = [];
        rBorder(:,k+1) = [];
        len = len-1;
    end
    k = k+1;
end
len = length(rBorder)-1;
k = 1;
while k < len
    if abs(rBorder(1,k+1) - rBorder(1,k)) < 0.01
        rBorder(:,k+1) = [];
        lBorder(:,k+1) = [];
        len = len-1;
    end
    k = k+1;
end
end

%------------- END CODE --------------