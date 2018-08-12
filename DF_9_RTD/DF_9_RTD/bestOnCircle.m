function newP=bestOnCircle(o1,r1,o2,r2,oldP)
syms x y
x1 = o1(:,1);
y1 = o1(:,2);
x2 = o2(:,1);
y2 = o2(:,2);
hp = oldP(:,3);
[x,y] = solve((x-x1)^2+(y-y1)^2-r1^2,(x-x2)^2+(y-y2)^2-r2^2);
p1 = vpa([x(1),y(1),hp],3);
p2 = vpa([x(2),y(2),hp],3);
if horizenDist(p1,oldP) < horizenDist(p2,oldP)
        newP = p1;
else
        newP = p2;
end

