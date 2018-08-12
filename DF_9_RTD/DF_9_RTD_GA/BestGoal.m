function goal=BestGoal(origin,goalBefore)
goal = goalBefore;
minDistCost = 1e6;
global noOfFlights
% noOfFlights = 9;
numOfTries = noOfFlights^4;
for n = 1:numOfTries
    randIndex = randperm(noOfFlights);
    goalNew = goalBefore(randIndex,:);
    distArray = zeros(1,noOfFlights);
    for i =1:noOfFlights
        distArray(i)  = horizenDist(origin(i,:),goalNew(i,:)); 
    end
    distSum = sum(distArray);
    distStd = std(distArray);
    distCost = distSum + distStd * noOfFlights;
    if distCost < minDistCost
        minDistCost = distCost
        distSum = distSum
        distStd = distStd
        goal = goalNew;
    end
end
