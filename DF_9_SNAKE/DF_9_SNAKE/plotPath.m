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