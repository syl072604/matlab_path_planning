% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clear
clc
global map source goal noOfFlights noOfDimensions splineSmoothing noOfPointsInSolution minlengthStdCost mincost minsegmentDistPenalty


% map = ones(1200,1200,600);
% noOfFlights = 36;

map = ones(600,600,300);
noOfFlights = 9;

noOfDimensions = 3;
noOfPointsInSolution=2; % no. of points that represent a candidate path, excluding the source and goal. each point marks a robot turn.
minlengthStdCost = ones(noOfFlights,1);
minsegmentDistPenalty = ones(noOfFlights,1);
mincost = 1e8;

% source=[50 50 100;
%         50 150 100;
%         50 250 100;
%         50 350 100;
%         50 450 100;
%         50 550 100;
%         50 650 100;
%         50 750 100;
%         50 850 100;
%         50 950 100;
%         50 1050 100;
%         50 1150 100;
%         600 50 100;
%         600 150 100;
%         600 250 100;
%         600 350 100;
%         600 450 100;
%         600 550 100;
%         600 650 100;
%         600 750 100;
%         600 850 100;
%         600 950 100;
%         600 1050 100;
%         600 1150 100;
%         1150 50 100;
%         1150 150 100;
%         1150 250 100;
%         1150 350 100;
%         1150 450 100;
%         1150 550 100;
%         1150 650 100;
%         1150 750 100;
%         1150 850 100;
%         1150 950 100;
%         1150 1050 100;
%         1150 1150 100;]; % source position in X, Y Z format
% goal=[350 350 200;
%       350 450 200;
%       350 550 200;
%       350 650 200;
%       350 750 200;
%       350 850 200;
%       450 350 200;
%       450 450 200;
%       450 550 200;
%       450 650 200;
%       450 750 200;
%       450 850 200;
%       550 350 200;
%       550 450 200;
%       550 550 200;
%       550 650 200;
%       550 750 200;
%       550 850 200;
%       650 350 200;
%       650 450 200;
%       650 550 200;
%       650 650 200;
%       650 750 200;
%       650 850 200;
%       750 350 200;
%       750 450 200;
%       750 550 200;
%       750 650 200;
%       750 750 200;
%       750 850 200;
%       850 350 200;
%       850 450 200;
%       850 550 200;
%       850 650 200;
%       850 750 200;
%       850 850 200;]; % goal position in X, Y Z format
source=[200 50 100;
        200 100 150;
        200 150 100;
        200 200 150;
        200 250 100;
        200 300 150;
        200 350 100;
        200 400 150;
        200 450 100;]; % source position in X, Y Z format
goal=[150 150 200;
      150 250 200;
      150 350 200;
      250 150 200;
      250 250 200;
      250 350 200;
      350 150 200;
      350 250 200;
      350 350 200;]; % goal position in X, Y Z format
  
minSGDistance = 1e6 * ones(noOfFlights);

goalTmp = goal;
goalReset = goal;

for i =1:noOfFlights
   minSGDistance = 1e6;
   indexOfGoal = 0;
   for j = 1: size(goalTmp,1)
        if DistanceCostDF(source(i,:),goalTmp(j,:)) < minSGDistance
              minSGDistance = DistanceCostDF(source(i,:),goalTmp(j,:)); 
              indexOfGoal = j;
        end        
   end
    goalReset(i,:) = goalTmp(indexOfGoal,:);  
    goalTmp(indexOfGoal,:) = [];
end
  
  
NoOfGenerations=200;
PopulationSize=1000;
splineSmoothing=true; % use spline based smoothing. the code has a dependence on the resoultion, and may be set to false if large changes in resolution are made.

%%%%% parameters end here %%%%%

tic;
for i = 1:noOfFlights
    if ~feasiblePointDF(source(i,:),map), error('source lies on an obstacle or outside map'); end
    if ~feasiblePointDF(goal(i,:),map), error('goal lies on an obstacle or outside map'); end
end

if noOfPointsInSolution<=0, error('noOfPointsInSolution should be greater than 1'); end

% currently the lower bounds and upper bounds are taken as 0 and 1 respectively, 
% these would be re-scaled in phenotype generation to ensure that they lie inside map. 
options=gaoptimset('PlotFcns', {@gaplotbestf, @gaplotdistance, @gaplotrange},'Generations',NoOfGenerations,'PopulationSize',PopulationSize, 'FitnessLimit', 10000);
[solution cost] = ga(@PathCostGADF, noOfPointsInSolution*noOfFlights*noOfDimensions,[],[],[],[],zeros(noOfPointsInSolution*noOfFlights*noOfDimensions,1),ones(noOfPointsInSolution*noOfFlights*noOfDimensions,1),[],options);
disp('click/press any key');
waitforbuttonpress; 
% if PathCostGADF(solution)>size(map,1)*size(map,2)*size(map,3) % indicating an infeasible path due to large cost due to penalties
%     error('no path found');
% end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,cost);

figure(2)
pathsp = [];
for i = 1:noOfFlights
    path = [source(i,:); [solution(3*i-2:noOfFlights*noOfDimensions:end)'*size(map,1) solution(3*i-1:noOfFlights*noOfDimensions:end)'*size(map,2) solution(3*i:noOfFlights*noOfDimensions:end)'*size(map,3)]; goal(i,:)]; % souce and goal is fixed. other points are from the GA individual representation
%     pathsp(:,:,i) =path;
    if splineSmoothing
       pathsp(:,:,i)=bsp(path); % a point based specification of path is smoothed by using splines
    end
    plot3(source(:,1),source(:,2),source(:,3),'*')
    plot3(goal(:,1),goal(:,2),goal(:,3),'o')
    hold on
%     plot3(pathsp(:,1),pathsp(:,2),pathsp(:,3),'-')
%     comet3(pathsp(:,1),pathsp(:,2),pathsp(:,3))
    hold on
end



