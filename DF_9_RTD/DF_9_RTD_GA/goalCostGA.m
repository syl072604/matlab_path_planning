% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function cost=goalCostGA(X) %GA cost function
%%
global source goalBefore noOfFlights 

%% build the path
cost=0;
path = 1: noOfFlights;
for i = 1:noOfFlights
    indexForChange = floor(X(i))+1;
    tmp = path(i);
    path(i) = path(indexForChange);
    path(indexForChange) = tmp;
end

%% cost
goalNew = goalBefore(path,:);
distArray = zeros(1,noOfFlights);
for i =1:noOfFlights
    distArray(i)  = horizenDist(source(i,:),goalNew(i,:)); 
end
distSum = sum(distArray);
distStd = std(distArray);
cost = distSum + distStd * noOfFlights * 0.5;



