figure(1);

vidObj = VideoWriter('path361.avi');
open(vidObj);

for i = 1:200
    
    for j = 1:36
        axis([0 1200 0 1200 0 600]);

        if i>150 
            plot3(pathsp(100,1,j),pathsp(100,2,j),pathsp(100,3,j),'*')
        elseif i>50
            plot3(pathsp(i-50,1,j),pathsp(i-50,2,j),pathsp(i-50,3,j),'*')
        else
            plot3(pathsp(1,1,j),pathsp(1,2,j),pathsp(1,3,j),'*')
        end
        
        hold on
    end
    m2(i) = getframe;
    writeVideo(vidObj,m2(i))
    hold off
end
movie(m2)
close(vidObj)