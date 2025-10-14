function [redundancyIntervalStr]=redundancyIntervalAnalysis(resultMatrix,robot)
% the result matrix is in the form MxNxP

%% $ redundancyIntervalStr $
% compute the feasibility of a single point i.e. if exist a combination of 
% M status and N tolerances in which the robot can achieve the motion trhough
% all the P poses. If exist, returns the interval of redundancy for each status
%in a structure


redundancyIntervalStr=struct('NoI',[],'value',[],'delimiters',[]);
J3_min = robot.qlim(3,1)*180/pi;
J3_max = robot.qlim(3,2)*180/pi;
J3_spacing = J3_min:1:J3_max;
% Quality of pose indices
Npose=size(resultMatrix,3);
sumMatrix=sum(resultMatrix,3);
for j=1:8
    possibleElements = sumMatrix(j,:)==Npose;
    possibleJ3=J3_spacing(possibleElements);
    diffJ3=diff(possibleJ3);
    
    %number of interval
    NOfInterval=nnz( diffJ3~=1)+1;
    [dim1,dim2,val]=find(diffJ3-1);
                       
    startInterval=1;
    redundancyIntervalStr(j).NoI= NOfInterval;
    for k=1:NOfInterval

        if k==NOfInterval
            interval=possibleJ3(startInterval:end);
            value=length(interval);
        else
            endInterval=dim2(k);
            interval=possibleJ3(startInterval:endInterval);
            value=length(interval);
            startInterval=dim2(k)+1;
        end
        redundancyIntervalStr(j).delimiters=[redundancyIntervalStr(j).delimiters; min(interval) max(interval);];
        redundancyIntervalStr(j).value=[redundancyIntervalStr(j).value value];
    end
end