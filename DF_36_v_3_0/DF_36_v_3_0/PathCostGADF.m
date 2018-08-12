% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function cost=PathCostGADF(X) %GA cost function
%%
global map source goal noOfFlights noOfDimensions noOfPointsInSolution minlengthStdCost mincost minsegmentDistPenalty

%% build the path
cost=0;
path = [];
distance = [];

for i = 1:noOfFlights
    path = [path;source(i,:); [X(3*i-2:noOfFlights*noOfDimensions:end)'*size(map,1) X(3*i-1:noOfFlights*noOfDimensions:end)'*size(map,2) X(3*i:noOfFlights*noOfDimensions:end)'*size(map,3)]; goal(i,:)]; % souce and goal is fixed. other points are from the GA individual representation
end

%% cost of path total length

for i = 1:noOfFlights
    distanceCost(i) = 0;
    segmentDistPenalty(i) =1;
    hgtCost(i) =1;

    for j=2:2+noOfPointsInSolution
        distance = DistanceCostDF(path((i-1)*(2+noOfPointsInSolution)+j-1,:),path((i-1)*(2+noOfPointsInSolution)+j,:));

        % 谋求最短路径
        distanceCost(i) = distanceCost(i) + distance;
        
        % 最低飞行高度不得低于50
        if path((i-1)*(2+noOfPointsInSolution)+j,3) <= 50
            hgtCost(i) = hgtCost(i) + 1000;
        end
        
        % 飞行轨迹不要有交叉
        for k = i+1:noOfFlights
            for m = 2:2+noOfPointsInSolution
                segmentDistPenalty(i) = segmentDistPenalty(i) + SegmentDistPenaltyDF(path((i-1)*(2+noOfPointsInSolution)+j-1,:),path((i-1)*(2+noOfPointsInSolution)+j,:),path((k-1)*(2+noOfPointsInSolution)+m-1,:),path((k-1)*(2+noOfPointsInSolution)+m,:));
            end
        end
    end

    cost = cost + segmentDistPenalty(i) *  distanceCost(i);
end

% 路径长度均衡
lengthStdCost = 1 + std(distanceCost)/100;

cost = cost * lengthStdCost;

if cost < mincost
    mincost = cost;
    minlengthStdCost = lengthStdCost;
    minsegmentDistPenalty = segmentDistPenalty;
end

