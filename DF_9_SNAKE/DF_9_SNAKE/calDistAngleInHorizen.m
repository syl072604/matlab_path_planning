function [dist, angle]=calDistAngleInHorizen(vectorAB,vectorBC)
vectorAB(3) = 0;
vectorBC(3) = 0;
dist = norm(vectorBC);
cosAngle = dot(vectorAB,vectorBC)/(norm(vectorAB)*norm(vectorBC));
angle = acos(cosAngle); % [0,pi]
crossABBC = cross(vectorAB,vectorBC);
if crossABBC(3) < 0
    direction = -1;  % clockwise
elseif crossABBC(3) > 0
    direction = 1;   % Counterclockwise
else
    direction = 0;  % not change direction
end

angle = angle * direction;

