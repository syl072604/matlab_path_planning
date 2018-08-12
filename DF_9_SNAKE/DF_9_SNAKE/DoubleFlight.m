
clear
clc
global map source goal noOfFlights noOfDimensions 

colorSpace = [1     0    0;
              0     1    0;
              0     0    1;
              1     1    0;
              1     0    1;
              0     1    1;
              0.6   0    1;
              1     0.5  0;
              0.5   0    0;];

map = ones(1200,1200,300);
noOfFlights = 36;
noOfDimensions = 3;

source = zeros(noOfFlights-1,2); % distance, cos of Angle
sourceInPosRegain = zeros(noOfFlights,3); % distance, cos of Angle, direction
goal = zeros(noOfFlights-1,2); % distance, cos of Angle
goalInPosRegain = zeros(noOfFlights,3); % distance, cos of Angle, direction
currentState = zeros(noOfFlights-1,3); % distance, cos of Angle, direction

% sourceInPos=[50 200 100;
%         100 200 100;
%         150 200 100;
%         200 200 100;
%         250 200 100;
%         300 200 100;
%         350 200 100;
%         400 200 100;
%         450 200 100;]; % source position in X, Y Z format
% goalInPos=[100 100 100;
%       200 100  100;
%       300 100  100;
%       100 200  200;
%       200 200  200;
%       300 200  200;
%       100 300  300;
%       200 300  300;
%       300 300 300;]; % goal position in X, Y Z format
sourceInPos=[100 50 100;
        100 150 100;
        100 250 100;
        100 350 100;
        100 450 100;
        100 550 100;
        100 650 100;
        100 750 100;
        100 850 100;
        100 950 100;
        100 1050 100;
        100 1150 100;
        200 50 100;
        200 150 100;
        200 250 100;
        200 350 100;
        200 450 100;
        200 550 100;
        200 650 100;
        200 750 100;
        200 850 100;
        200 950 100;
        200 1050 100;
        200 1150 100;
        300 50 100;
        300 150 100;
        300 250 100;
        300 350 100;
        300 450 100;
        300 550 100;
        300 650 100;
        300 750 100;
        300 850 100;
        300 950 100;
        300 1050 100;
        300 1150 100;]; % source position in X, Y Z format
goalInPos=[350 350 200;
      350 450 200;
      350 550 200;
      350 650 200;
      350 750 200;
      350 850 200;
      450 850 250;
      450 750 250;
      450 650 250;
      450 550 250;
      450 450 250;
      450 350 250;
      550 350 300;
      550 450 300;
      550 550 300;
      550 650 300;
      550 750 300;
      550 850 300;
      650 850 350;
      650 750 350;
      650 650 350;
      650 550 350;
      650 450 350;
      650 350 350;
      750 350 400;
      750 450 400;
      750 550 400;
      750 650 400;
      750 750 400;
      750 850 400;
      850 850 450;
      850 750 450;
      850 650 450;
      850 550 450;
      850 450 450;
      850 350 450;]; % goal position in X, Y Z format
  
PointOsource = sourceInPos(1,:);

for i = 1:noOfFlights-1
    if i == 1
        vectorAB = [1 0 0];
    else
        vectorAB = sourceInPos(i,:) - sourceInPos(i-1,:);
    end
    
    vectorBC = sourceInPos(i+1,:) - sourceInPos(i,:);
    [dist, angle] = calDistAngleInHorizen(vectorAB,vectorBC);
    source(i,:) = [dist angle];
end

PointOgoal = goalInPos(1,:);
for i = 1:noOfFlights-1
    if i == 1
        vectorAB = [1 0 0];
    else
        vectorAB = goalInPos(i,:) - goalInPos(i-1,:);
    end
    
    vectorBC = goalInPos(i+1,:) - goalInPos(i,:);
    [dist, angle] = calDistAngleInHorizen(vectorAB,vectorBC);    
    goal(i,:) = [dist angle];
end

% goalInPosRegain = calPathFromDistAngle(PointOgoal, goal, noOfFlights)


distChangePerStep = 50;
angelChangePerStep = pi/10; 
 
tic;
for i = 1:noOfFlights
    if ~feasiblePointDF(sourceInPos(i,:),map), error('source lies on an obstacle or outside map'); end
    if ~feasiblePointDF(sourceInPos(i,:),map), error('goal lies on an obstacle or outside map'); end
end

stepNo = 1;
currentState = source;
goalNotReached = 1;

pathX(:,stepNo) = sourceInPos(:,1);
pathY(:,stepNo) = sourceInPos(:,2);
pathZ(:,stepNo) = sourceInPos(:,3);

while goalNotReached
    goalNotReached = 0;
    stepNo = stepNo + 1;
    for i = 1:noOfFlights-1
       
        [distDiff, angleDiff]=stateDiffAB(currentState(i,:), goal(i,:));
        
        if abs(distDiff) < distChangePerStep && abs(angleDiff) < angelChangePerStep
            currentState(i,:) = goal(i,:);
            continue;
        else
            goalNotReached = 1;
            if abs(distDiff) > distChangePerStep            
                currentState(i,1) = currentState(i,1) + distDiff / abs(distDiff) * distChangePerStep;
            else
                currentState(i,1) = goal(i,1);
            end
            
            if abs(angleDiff) > angelChangePerStep
               currentState(i,2) = currentState(i,2) + angleDiff / abs(angleDiff) * angelChangePerStep;
            else
               currentState(i,2) = goal(i,2);
            end
        end
    end
    pathThisStep = calPathFromDistAngle(PointOsource, currentState, noOfFlights);
    pathX(:,stepNo) = pathThisStep(:,1);
    pathY(:,stepNo) = pathThisStep(:,2);
    pathZ(:,stepNo) = pathThisStep(:,3);
    

%     for k = 1:noOfFlights
%         patha = [pathX(k,stepNo-1:stepNo)' pathY(k,stepNo-1:stepNo)' pathZ(k,stepNo-1:stepNo)'];
%         axis([0 800 0 800 0 300]);
%         figure(2)
%         plot3(patha(:,1),patha(:,2),patha(:,3),'linewidth',2,'color',colorSpace(mod(k,9)+1,:))
%         hold on
%         plot3(patha(2,1),patha(2,2),patha(2,3),'*','color',colorSpace(mod(k,9)+1,:))
%         hold on
%     end
%     hold off
  
end


vidObj = VideoWriter('path361snake.avi');
vidObj.FrameRate = 5;
open(vidObj);
figure(1)
plot3(pathX(:,1), pathY(:,1), pathZ(:,1));
hold on
plot3(pathX(:,1), pathY(:,1), pathZ(:,1),'*');
axis([-500 2000 -500 2000 0 200]);
hold off
m2(1) = getframe;
writeVideo(vidObj,m2(1))
im=frame2im(m2(1));
[I,mappp]=rgb2ind(im,256);

filename = 'path361snake.gif'
imwrite(I,mappp,filename,'gif','Loopcount',inf,...
            'DelayTime',0.3);
for k = 1:stepNo
    
    plot3(pathX(:,k), pathY(:,k), pathZ(:,k));
    hold on
    plot3(pathX(:,k), pathY(:,k), pathZ(:,k),'*');
    axis([-500 2000 -500 2000 0 200]);
    hold off
    m2(stepNo) = getframe;
    writeVideo(vidObj,m2(stepNo))
        im=frame2im(m2(stepNo));
    [I,mappp]=rgb2ind(im,256);
    imwrite(I,mappp,filename,'gif','WriteMode','append',...
            'DelayTime',0.3);
end
movie(m2)
close(vidObj)


