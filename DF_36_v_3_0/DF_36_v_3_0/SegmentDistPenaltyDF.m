% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function penalty=SegmentDistPenaltyDF(pointA1,pointA2,pointB1,pointB2)
segmentDistance  = SegmentDistDF(pointA1,pointA2,pointB1,pointB2);

if segmentDistance<50
%     penalty = (segmentDistance/100)^(-4)-1;
    penalty = 1000;
elseif   segmentDistance<100
    penalty = 5;
else
    penalty = 0;
end
