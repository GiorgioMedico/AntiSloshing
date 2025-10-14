function plotFeasibilityIndex(resultMatrix,robot)
% the result matrix is in the form MxNxP

% Plot the feasibility of a single point i.e. the combination of 
% M status and N tolerances in which the robot can achieve the motion trhough
% all the P poses. Robot poses are represented in blu (start) and red
% (end). The orange line represent the zone in which all the poses exist.
Npose=size(resultMatrix,3);

startDistance=1;
distanceStep=0.05;
overallDistance=1.2;
graphHeigth=startDistance:-distanceStep:startDistance-distanceStep*Npose;
J3_min = robot.qlim(3,1)*180/pi;
J3_max = robot.qlim(3,2)*180/pi;
J3_spacing = J3_min:1:J3_max;

figure('units','normalized','outerposition',[0 0 1 1]);
sumMatrix=sum(resultMatrix,3);
colorStr=['b' 'r' 'b' 'r' 'b' 'r' 'b' 'r' 'b' 'r' 'b' 'r' 'b' 'r' 'b' 'r' ];

for j=1:8
    subplot(2,4,j)
    grid on
    hold on
    for p=1:Npose
        plot(J3_spacing,resultMatrix(j,:,p)    *graphHeigth(p), '.','MarkerSize',15,'Color',colorStr(p));
    end
    xlabel('redundancy')
    ylabel(['status' num2str(j-1)])
    ylim([0 1.3])
    xticks(J3_min:50:J3_max)
    possibleElements= sumMatrix(j,:)==Npose;
    plot(J3_spacing,possibleElements*overallDistance,'.','Color',[255/255 161/255 0/255],'MarkerSize',20)    
end

