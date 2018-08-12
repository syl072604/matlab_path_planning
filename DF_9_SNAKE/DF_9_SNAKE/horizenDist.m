function h=horizenDist(a,b)
a(:,3) = b(:,3);
h = sqrt(sum((a-b).^2));