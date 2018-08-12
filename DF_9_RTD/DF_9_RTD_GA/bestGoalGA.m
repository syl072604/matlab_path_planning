function goal=bestGoalGA(origin,goalBefore)
global noOfFlights
NoOfGenerations=50;
PopulationSize=20000;
% noOfFlights = 9;
tic;
% currently the lower bounds and upper bounds are taken as 0 and 1 respectively, 
% these would be re-scaled in phenotype generation to ensure that they lie inside map. 
options=gaoptimset('PlotFcns', {@gaplotbestf, @gaplotdistance, @gaplotrange},'Generations',NoOfGenerations,'PopulationSize',PopulationSize, 'FitnessLimit', 5000);
[solution,cost] = ga(@goalCostGA, noOfFlights,[],[],[],[],zeros(noOfFlights,1),ones(noOfFlights,1)*noOfFlights-0.01,[],options);
disp('click/press any key');
waitforbuttonpress; 
% if PathCostGADF(solution)>size(map,1)*size(map,2)*size(map,3) % indicating an infeasible path due to large cost due to penalties
%     error('no path found');
% end
fprintf('processing time=%d \n Path Length=%d \n', toc,cost);

path = 1: noOfFlights;
for i = 1:noOfFlights
    indexForChange = floor(solution(i))+1;
    tmp = path(i);
    path(i) = path(indexForChange);
    path(indexForChange) = tmp;
end
goal = goalBefore(path,:)
distArray = zeros(1,noOfFlights);
for i =1:noOfFlights
    distArray(i)  = horizenDist(origin(i,:),goal(i,:)); 
end
distSum = sum(distArray)
distStd = std(distArray)
distCost = distSum + distStd * noOfFlights * 0.5


