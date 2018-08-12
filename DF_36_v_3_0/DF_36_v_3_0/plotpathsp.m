for i = 1:noOfFlights
    pathaa = [source(i,:); [solution(3*i-2:noOfFlights*noOfDimensions:end)'*size(map,1) solution(3*i-1:noOfFlights*noOfDimensions:end)'*size(map,2) solution(3*i:noOfFlights*noOfDimensions:end)'*size(map,3)]; goal(i,:)]; % souce and goal is fixed. other points are from the GA individual representation

%     pathspaa=bsp(pathaa); % a point based specification of path is smoothed by using splines
    pathspaa = pathaa;
    plot3(source(:,1),source(:,2),source(:,3),'*')
    plot3(goal(:,1),goal(:,2),goal(:,3),'o')
    hold on
    plot3(pathspaa(:,1),pathspaa(:,2),pathspaa(:,3),'linewidth', 5)
%     comet3(pathsp(:,1),pathsp(:,2),pathsp(:,3))
    hold on
end