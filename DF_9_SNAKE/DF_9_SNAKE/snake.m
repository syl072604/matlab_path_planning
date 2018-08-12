noOfFlights = 9;
noOfDimentions = 3;
goal = zeros(noOfFlights,noOfDimentions);

for i = 1:noOfFlights
    goal(i,:) = floor(rand(1,3)*10)*100;
end
goalOrigin = goal;
goalSortedByX = sortrows(goal,1);

plot3(goalSortedByX(:,1),goalSortedByX(:,2),goalSortedByX(:,3),'o')
hold on
plot3(goalSortedByX(:,1),goalSortedByX(:,2),goalSortedByX(:,3),'-')

