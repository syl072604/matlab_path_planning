function pointC=calPosFromAngle(vectorAB,dist,angle,pointB)
%         C
%         >
%       / |
%      /  |
% ---->--->
% A   B   D

if angle == 0
    direction =0;
else
    direction = angle / abs(angle);
end

angle = abs(angle);
cosAngle = cos(angle);
sinAngle = sin(angle);

vectorBD = dist * cosAngle / norm(vectorAB) * vectorAB;
vectorZ =[0 0 direction];

vectorDC = cross(vectorZ, vectorAB) / norm(vectorAB) * dist * sinAngle;
vectorBC = vectorBD + vectorDC;
pointC = pointB + vectorBC;


