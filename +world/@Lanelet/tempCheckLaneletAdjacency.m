function [notParallel,distanceLK,distanceRK] = tempCheckLaneletAdjacency(l_k,r_k,j1,j2,length,widthK,widthJ)
% tempChecklaneletAdjacency - check adjacency, temporary work around
%need to be deleted later
% Syntax:
%   [notParallel,distanceLK,distanceRK] =
%   checkLaneletAdjacency(l_k,r_k,j1,j2,length,widthK,widthJ);
%
% Inputs:
%   l_k -left border of K
%   r_k - right border of k
%   j1 and j2 - left and roght border of J based on direction
%   length - length of lanelet
%   widthK and widthJ - width of lane K and J
%
% Outputs:
%   notParallel - if lanes are not parallel
%   distanceLK and distanceRK - distance from the left and right border of K 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Rajat Koner
% Written:      22 JAN 2017
% Last update:
%
% Last revision:---

%------------- BEGIN CODE --------------
%initialize variable
notParallel = 0;
%distance between left & right border of K and border of J
distanceLKJ1 =0;distanceRKJ2=0;
%initillize pivoting element for deviation
distanceLKJ1_Pivot =0;distanceRKJ2_Pivot=0;

%initillize flag to set pivot for deviation checking
isDistanceLKJ1_PivotSet =0;isDistanceRKJ2_PivotSet=0;

distanceLK=0;distanceRK=0;

for t= 1:2    
    if(t==1)        
    %1st of left border of K and right or left border of J based on direction
    distanceLKJ1(t)=sqrt((l_k(1,t) - j1(1,t))^2 + ...
        (l_k(2,t) - j1(2,t))^2);
    %1st of right border of K and left or right border of J based on direction
    distanceRKJ2(t)=sqrt((r_k(1,t) - j2(1,t))^2 + ...
        (r_k(2,t) - j2(2,t))^2);
    else
        distanceLKJ1(t)=sqrt((l_k(1,end) - j1(1,end))^2 + ...
        (l_k(2,end) - j1(2,end))^2);
    %end of right border of K and left or right border of J based on direction
    distanceRKJ2(t)=sqrt((r_k(1,end) - j2(1,end))^2 + ...
        (r_k(2,end) - j2(2,end))^2);
    end
    %set pivot
    if distanceLKJ1(t)>0.5 && isDistanceLKJ1_PivotSet ==0
        distanceLKJ1_Pivot = distanceLKJ1(t);
        isDistanceLKJ1_PivotSet=1;
    end
    
    
    %set pivot
    if distanceRKJ2(t)>0.5 && isDistanceRKJ2_PivotSet ==0
        distanceRKJ2_Pivot = distanceRKJ2(t);
        isDistanceRKJ2_PivotSet=1;
    end
    
    
    %check devation with pivot ,sudn't be more than 5% of border distance ,else break loop
    if(t>1)
        if abs(distanceLKJ1(1) -distanceLKJ1(2)) > 0.5
            notParallel=1;
            break;
        end
        if abs(distanceRKJ2(1) -distanceRKJ2(2))> 0.5
            notParallel=1;
            break;
        end
    end
    %summing up border distance node wise
    distanceLK =distanceLKJ1(t)+ distanceLK;
    distanceRK = distanceRKJ2(t)+ distanceRK;
end  %end of for loop

% total width of the two road
totalWidth = widthJ + widthK;  
%taking the average border distance of lane k and J
distanceRK = (distanceRK/t)/totalWidth;
distanceLK = (distanceLK/t)/totalWidth;

end
%------------- END CODE --------------