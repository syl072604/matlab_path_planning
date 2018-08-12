function y=SegmentDistDF(pointA1,pointA2,pointB1,pointB2)
vectorA=pointA2-pointA1;
vectorB=pointB2-pointB1;
unitA = vectorA ./norm(vectorA);
unitB = vectorB ./norm(vectorB);

pointA1Delta = pointA1 + 10 * unitA;
pointA2Delta = pointA2 - 10 * unitA;

pointB1Delta = pointB1 + 10 * unitB;
pointB2Delta = pointB2 - 10 * unitB;

vectorC=cross(vectorA,vectorB);


vectorA1B1 = pointB1 - pointA1;
vectorA1DeltaB1 = pointB1 - pointA1Delta;
vectorA2B1 = pointB1 - pointA2;
vectorA2DeltaB1 = pointB1 - pointA2Delta;

vectorA2B2 = pointB2 - pointA2;

vectorB1A1 = pointA1 - pointB1;
vectorB1DeltaA1 = pointA1 - pointB1Delta;
vectorB2A1 = pointA1 - pointB2;
vectorB2DeltaA1 = pointA1 - pointB2Delta;




A1toVectorB = norm(cross(vectorA1B1,vectorB))./norm(vectorB);
A1DeltatoVectorB = norm(cross(vectorA1DeltaB1,vectorB))./norm(vectorB);

A2toVectorB = norm(cross(vectorA2B1,vectorB))./norm(vectorB);
A2DeltatoVectorB = norm(cross(vectorA2DeltaB1,vectorB))./norm(vectorB);

B1toVectorA = norm(cross(vectorB1A1,vectorA))./norm(vectorA);
B1DeltatoVectorA = norm(cross(vectorB1DeltaA1,vectorA))./norm(vectorA);

B2toVectorA = norm(cross(vectorB2A1,vectorA))./norm(vectorA);
B2DeltatoVectorA = norm(cross(vectorB2DeltaA1,vectorA))./norm(vectorA);

% 判断是否两条线段之间的最短距离不是他们的垂线段，也就是说两条直线的垂线段是在某条线段的延长线上。
if A1toVectorB < A1DeltatoVectorB
    if B1toVectorA < B1DeltatoVectorA
        y = norm(vectorB1A1);
    elseif  B2toVectorA < B2DeltatoVectorA
        y = norm(vectorB2A1);
    else
        y = A1toVectorB;
    end
elseif A2toVectorB < A2DeltatoVectorB
    if B1toVectorA < B1DeltatoVectorA
        y = norm(vectorA2B1);
    elseif  B2toVectorA < B2DeltatoVectorA
        y = norm(vectorA2B2);
    else
        y = A2toVectorB;
    end
else
    if B1toVectorA < B1DeltatoVectorA
        y = B1toVectorA;
    elseif  B2toVectorA < B2DeltatoVectorA
        y = B2toVectorA;
    else
        if norm(vectorC)~=0
            y = norm(dot(vectorC,vectorA2B2))./norm(vectorC);
        else
            y = norm(cross(vectorA,vectorA2B2))./norm(vectorA);
        end
    end
end









