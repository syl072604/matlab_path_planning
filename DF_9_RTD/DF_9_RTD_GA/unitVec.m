function y=unitVec(pointA,pointB)
vectorAB=pointB-pointA;
y = vectorAB/norm(vectorAB);
