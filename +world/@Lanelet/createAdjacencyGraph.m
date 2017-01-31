function createAdjacencyGraph(lanelets)
% createAdjacencyGraph - Summary of this function goes here
%
% Syntax:
%   createAdjacencyGraph(lanelets);
%
% Inputs:
%   lanelets - array on lanelet objects
%
% Outputs:
%   none
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:       Rajat Koner
% Written:      8-December-2016
% Last update:  17-January-2016
%
% Last revision:---

%------------- BEGIN CODE --------------

epsilion =0.27;                %thresold value of parallel lanelet
numLanelets = length(lanelets);
if numLanelets > 1
    % Adjecency Graph can only be created for multiple lanelet
    for k = 1:numLanelets
        for j = 1:numLanelets
            if k ~= j
                if lanelets(k).leftBorderVertices(:,end) == lanelets(j).leftBorderVertices(:,1)
                    if lanelets(k).rightBorderVertices(:,end) == lanelets(j).rightBorderVertices(:,1)
                        lanelets(k).addAdjacentLanelet('successor', lanelets(j));
                        lanelets(j).addAdjacentLanelet('predecessor', lanelets(k));
                    end
                end
                
                l_k = lanelets(k).leftBorderVertices;
                r_k = lanelets(k).rightBorderVertices;
                r_j = lanelets(j).rightBorderVertices;
                l_j = lanelets(j).leftBorderVertices;
                % changes made in this function are temporary
               % if (length(l_k)==length(l_j) && length(r_k)==length(r_j))           %length of each adjacent vertices has to be equal
                   if(isempty(lanelets(k).adjacentLeft) && isempty(lanelets(j).adjacentRight))                   
                        widthK =0;widthJ=0;
                        noOfVertices = length(lanelets(k).leftBorderVertices);
                       % for t= 1:noOfVertices
                            %width of lane k and j for 1st and last
                            %vertices
                            widthK= sqrt((l_k(1,1) - r_k(1,1))^2 + ...
                                (l_k(2,1) - r_k(2,1))^2);
                            widthK= widthK + sqrt((l_k(1,end) - r_k(1,end))^2 + ...
                                (l_k(2,end) - r_k(2,end))^2);                            
                            widthJ = sqrt((l_j(1,1) - r_j(1,1))^2 + ...
                                (l_j(2,1) - r_j(2,1))^2);
                             widthJ = widthJ + sqrt((l_j(1,end) - r_j(1,end))^2 + ...
                                (l_j(2,end) - r_j(2,end))^2);
                        %end
                        %taking average width of the road
                        widthK = widthK/2;
                        widthJ = widthJ/2;
                        
                        %check adjacency for same direction
                        [notParallelSame,distanceLR,distanceRL] =  world.Lanelet.tempCheckLaneletAdjacency(l_k,r_k,r_j,l_j,noOfVertices,widthK,widthJ);
                        
                        %check adjacency for opposite direction
                        [notParallelOpposite,distanceLL,distanceRR] =  world.Lanelet.tempCheckLaneletAdjacency(l_k,r_k,fliplr(l_j),fliplr(r_j),noOfVertices,widthK,widthJ);
                        
                        %break loop if both same and opposite are not
                        %parallel
                        if notParallelSame == 1 && notParallelOpposite==1
                            continue;
                        end                                            
                        
                        if(notParallelSame==0)
                            %left parallel lanelet
                            if(distanceLR <= epsilion)
                                lanelets(k).addAdjacentLanelet('adjacentLeft', {lanelets(j), 'same'});
                                lanelets(j).addAdjacentLanelet('adjacentRight', {lanelets(k), 'same'});
                            end
                                                        
                            %right parallel lanelet
                            if(distanceRL <= epsilion)
                                lanelets(k).addAdjacentLanelet('adjacentRight', {lanelets(j), 'same'});
                                lanelets(j).addAdjacentLanelet('adjacentLeft', {lanelets(k), 'same'});
                            end
                         end
                        
                        
                        if(notParallelOpposite==0)
                            %left parallel opposite lanelet
                            if(distanceLL <= epsilion)
                                lanelets(k).addAdjacentLanelet('adjacentLeft', {lanelets(j), 'opposite'});
                                lanelets(j).addAdjacentLanelet('adjacentLeft', {lanelets(k), 'opposite'});
                            end
                            
                            %right parallel opposite lanelet
                            if(distanceRR <= epsilion)
                                lanelets(k).addAdjacentLanelet('adjacentRight', {lanelets(j), 'opposite'});
                                lanelets(j).addAdjacentLanelet('adjacentRight', {lanelets(k), 'opposite'});
                            end
                        end
                    end
                    
               % end  %end of check equal no of vertices
            end
        end
        
    end
    
end
end

%------------- END CODE --------------

