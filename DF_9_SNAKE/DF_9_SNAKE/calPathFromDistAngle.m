function pathInPos=calPathFromDistAngle(originPoint, pathInDistAngle, noOfFlights)
pathInPos(1,:) = originPoint;
for i = 1:noOfFlights-1
    if i == 1
        vectorAB = [1 0 0];
    else
        vectorAB = pathInPos(i,:) - pathInPos(i-1,:);
    end
    pointB = pathInPos(i,:);
    
    dist = pathInDistAngle(i,1);
    angle = pathInDistAngle(i,2);
    
    pointC = calPosFromAngle(vectorAB,dist,angle,pointB);
    pathInPos(i+1,:) = pointC;
end