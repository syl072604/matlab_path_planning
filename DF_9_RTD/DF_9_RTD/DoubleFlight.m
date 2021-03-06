% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clear
clc
global map source goal noOfFlights noOfDimensions splineSmoothing

colorSpace = [1     0    0;
              0     1    0;
              0     0    1;
              1     1    0;
              1     0    1;
              0     1    1;
              0.6   0    1;
              1     0.5  0;
              0.5   0    0;];

map = ones(1200,1200,600);
noOfFlights = 36;
noOfDimensions = 3;

source=[50 50 100;
        50 150 100;
        50 250 100;
        50 350 100;
        50 450 100;
        50 550 100;
        50 650 100;
        50 750 100;
        50 850 100;
        50 950 100;
        50 1050 100;
        50 1150 100;
        600 50 100;
        600 150 100;
        600 250 100;
        600 350 100;
        600 450 100;
        600 550 100;
        600 650 100;
        600 750 100;
        600 850 100;
        600 950 100;
        600 1050 100;
        600 1150 100;
        1150 50 100;
        1150 150 100;
        1150 250 100;
        1150 350 100;
        1150 450 100;
        1150 550 100;
        1150 650 100;
        1150 750 100;
        1150 850 100;
        1150 950 100;
        1150 1050 100;
        1150 1150 100;]; % source position in X, Y Z format
goalBeforeReset=[350 350 200;
      350 450 200;
      350 550 200;
      350 650 200;
      350 750 200;
      350 850 200;
      450 350 250;
      450 450 250;
      450 550 250;
      450 650 250;
      450 750 250;
      450 850 250;
      550 350 300;
      550 450 300;
      550 550 300;
      550 650 300;
      550 750 300;
      550 850 300;
      650 350 350;
      650 450 350;
      650 550 350;
      650 650 350;
      650 750 350;
      650 850 350;
      750 350 400;
      750 450 400;
      750 550 400;
      750 650 400;
      750 750 400;
      750 850 400;
      850 350 450;
      850 450 450;
      850 550 450;
      850 650 450;
      850 750 450;
      850 850 450;]; % goal position in X, Y Z format

% source=[200 50 100;
%         200 100 150;
%         200 150 100;
%         200 200 150;
%         200 250 100;
%         200 300 150;
%         200 350 100;
%         200 400 150;
%         200 450 100;]; % source position in X, Y Z format
% goalBeforeReset=[100 100 100;
%       100 250 100;
%       100 400 100;
%       250 100 200;
%       250 250 200;
%       250 400 200;
%       400 100 300;
%       400 250 300;
%       400 400 300;]; % goal position in X, Y Z format
  
minSGDistance = 1e6 * ones(noOfFlights);

goalTmp = goalBeforeReset;
goal = goalBeforeReset;

for i =1:noOfFlights
   minSGDistance = 1e6;
   indexOfGoal = 0;
   for j = 1: size(goalTmp,1)
        if DistanceCostDF(source(i,:),goalTmp(j,:)) < minSGDistance
              minSGDistance = DistanceCostDF(source(i,:),goalTmp(j,:)); 
              indexOfGoal = j;
        end        
   end
    goal(i,:) = goalTmp(indexOfGoal,:);  
    goalTmp(indexOfGoal,:) = [];
end
  
currentPos = source;
stepSizeHorizen = 40;
stepSizeVertical = 30;
safeRange = 50;

acceptRadius = 50;

%%%%% parameters end here %%%%%

tic;
for i = 1:noOfFlights
    if ~feasiblePointDF(source(i,:),map), error('source lies on an obstacle or outside map'); end
    if ~feasiblePointDF(goal(i,:),map), error('goal lies on an obstacle or outside map'); end
end

stepNo = 1;

pathX(:,stepNo) = source(:,1);
pathY(:,stepNo) = source(:,2);
pathZ(:,stepNo) = source(:,3);

goalNotReached = 1;

figure(1)
vidObj = VideoWriter('path363.avi');
vidObj.FrameRate = 5;
open(vidObj);
for k = 1:noOfFlights

    axis([0 1200 0 1200 0 600]);
    plot3(source(k,1),source(k,2),source(k,3),'o','color',colorSpace(mod(k,9)+1,:))
    plot3(goal(k,1),goal(k,2),goal(k,3),'*','color',colorSpace(mod(k,9)+1,:))
    hold on
end
m2(stepNo) = getframe;
writeVideo(vidObj,m2(stepNo))
im=frame2im(m2(stepNo));
[I,mappp]=rgb2ind(im,256);

filename = 'path363.gif'
imwrite(I,mappp,filename,'gif','Loopcount',inf,...
            'DelayTime',0.1);

while goalNotReached
    
    for k = 1:noOfFlights
        axis([0 1200 0 1200 0 600]);
        
        plot3(goal(k,1),goal(k,2),goal(k,3),'*','color',colorSpace(mod(k,9)+1,:))
        hold on
    end
    
    goalNotReached = 0;

    stepNo = stepNo + 1    
    for i = 1:noOfFlights
        if(DistanceCostDF(currentPos(i,:),goal(i,:))) < acceptRadius
            
            pathX(i,stepNo) = goal(i,1);
            pathY(i,stepNo) = goal(i,2);
            pathZ(i,stepNo) = goal(i,3);
            
            continue;
        else
            goalNotReached = 1;
            
            unitVecToGoal = unitVec(currentPos(i,:),goal(i,:));
            if horizenDist(currentPos(i,:),goal(i,:)) < stepSizeHorizen
                desiredPosNextStep = goal(i,:);
                desiredPosNextStep(:,3) = currentPos(i,3) + stepSizeVertical * unitVecToGoal(3);
            elseif abs(currentPos(i,3)-goal(i,3)) < stepSizeVertical
                desiredPosNextStep = currentPos(i,:) + stepSizeHorizen * unitVecToGoal;
                desiredPosNextStep(:,3) = goal(i,3);
            else
                desiredPosNextStep = currentPos(i,:) + stepSizeHorizen * unitVecToGoal;
                desiredPosNextStep(:,3) = currentPos(i,3) + stepSizeVertical * unitVecToGoal(3);
                
            end

            disturbFromFlight = i;
            failedPoint = false;
            while disturbFromFlight ~= 0
                disturbFromFlight = i;
                for j = 1:noOfFlights
                    if j ~= i
                        if horizenDist(desiredPosNextStep,currentPos(j,:)) < (safeRange - 0.5 * abs(desiredPosNextStep(3)-currentPos(j,3)))
                            disturbFromFlight = j;
                            break;
                        end
                    end
                end
                if disturbFromFlight  == i
                    disturbFromFlight = 0;
                else
                    
                    if failedPoint
                        desiredPosNextStep = randPoint(currentPos(i,:),stepSizeHorizen,stepSizeVertical,desiredPosNextStep);
                    else
                        desiredPosNextStep = bestOnCircle(currentPos(i,:),stepSizeHorizen,currentPos(disturbFromFlight,:),safeRange+5,desiredPosNextStep);
                    end
                    failedPoint = true;
                end
            end
            currentPos(i,:) = desiredPosNextStep;
            pathX(i,stepNo) = currentPos(i,1);
            pathY(i,stepNo) = currentPos(i,2);
            pathZ(i,stepNo) = currentPos(i,3);
        end
    end

    pause(1);
    
    if stepNo >1
        for k = 1:noOfFlights
            patha = [pathX(k,stepNo-1:stepNo)' pathY(k,stepNo-1:stepNo)' pathZ(k,stepNo-1:stepNo)'];
            axis([0 1200 0 1200 0 600]);
            plot3(patha(:,1),patha(:,2),patha(:,3),'linewidth',2,'color',colorSpace(mod(k,9)+1,:))
            hold on
            plot3(patha(2,1),patha(2,2),patha(2,3),'o','color',colorSpace(mod(k,9)+1,:))
            hold on
        end
    end
     hold off
    m2(stepNo) = getframe;
    writeVideo(vidObj,m2(stepNo))
    im=frame2im(m2(stepNo));
    [I,mappp]=rgb2ind(im,256);
    imwrite(I,mappp,filename,'gif','WriteMode','append',...
            'DelayTime',0.1);

  
end
movie(m2)
close(vidObj)


splineSmoothing=false; % use spline based smoothing. the code has a dependence on the resoultion, and may be set to false if large changes in resolution are made.

% disp('click/press any key');
% waitforbuttonpress; 
cost = 0;

fprintf('processing time=%d \nPath Length=%d \n\n', toc,cost);
figure(2)
plot3(goal(:,1),goal(:,2),goal(:,3),'*')
hold on
plot3(source(:,1),source(:,2),source(:,3),'o')
hold on
for k = 1:noOfFlights
    patha = [pathX(k,:)' pathY(k,:)' pathZ(k,:)'];
    pathspa = bsp(patha);
    axis([0 1200 0 1200 0 600]);

    plot3(pathspa(:,1),pathspa(:,2),pathspa(:,3))
    hold on
end



