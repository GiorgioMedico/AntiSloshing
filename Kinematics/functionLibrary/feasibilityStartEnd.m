function [sol_start,sol_finish]=feasibilityStartEnd(startPose,endPose,robot,varargin)

% [sol_start,sol_finish]=feasibilityStartEnd(startPose,endPose,robot,GRAPHICS)
%
% Compute feasibility of startPose and endPose. sol_start/sol_finish are a 
%cell arrays, % sol{i} contains the feasibility vector of status i, in the 
%redundancy range
% 
%
%GRAPHICS= 0,1 


if startPose == endPose
sol_start = lbr_kinematic_capabilities(robot,startPose);
sol_finish = sol_start;

else
sol_start = lbr_kinematic_capabilities(robot,startPose);
sol_finish = lbr_kinematic_capabilities(robot,endPose);
end


%GRAPHICS
graphics=varargin{1};
if graphics
plotFeasibilityIndex(sol_start,sol_finish,robot);
end

