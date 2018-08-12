function newP=randPoint(currentP,stepSizeHorizen,stepSizeVertical,oldP)
alpha = 2*pi*rand;
newP = currentP + [stepSizeHorizen*cos(alpha),stepSizeHorizen*sin(alpha), 0];
if rand<0.7
    newP(:,3) = oldP(:,3);
else
    deltaH = (2*rand-1) * stepSizeVertical;
    newP(:,3) = oldP(:,3) + deltaH;
end


