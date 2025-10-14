% Creata da Claudio Mazzotti

%     KUKA_X = GoalPoint(1);%mm
%     KUKA_Y = GoalPoint(2);%mm
%     KUKA_Z = GoalPoint(3);%mm
%     KUKA_A = GoalPoint(4);%gradi
%     KUKA_B = GoalPoint(5);%gradi
%     KUKA_C = GoalPoint(6);%gradi

function plot_lbr_kinematic_capabilities(robot,GoalPoint)

J3_min = robot.qlim(3,1)*180/pi;
J3_max = robot.qlim(3,2)*180/pi;

J3_spacing = [J3_min:1:J3_max];


for i = 1:1:length(J3_spacing)
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),0))
        sol{1}(i) = 0;
    else
        sol{1}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),1))
        sol{2}(i) = 0;
            else
        sol{2}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),2))
        sol{3}(i) = 0;
            else
        sol{3}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),3))
        sol{4}(i) = 0;
            else
        sol{4}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),4))
        sol{5}(i) = 0;
            else
        sol{5}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),5))
        sol{6}(i) = 0;
            else
        sol{6}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),6))
        sol{7}(i) = 0;
            else
        sol{7}(i) = 1;
    end
    
    if isnan(inverseKinematicLBR_J(robot,GoalPoint,J3_spacing(i),7))
        sol{8}(i) = 0;
            else
        sol{8}(i) = 1;
    end
    clc
    
end

figure
plot(J3_spacing,sol{1},'*')
xlabel('redundancy')
ylabel('status 0')

figure
plot(J3_spacing,sol{2},'*')
xlabel('redundancy')
ylabel('status 1')

figure
plot(J3_spacing,sol{3},'*')
xlabel('redundancy')
ylabel('status 2')

figure
plot(J3_spacing,sol{4},'*')
xlabel('redundancy')
ylabel('status 3')

figure
plot(J3_spacing,sol{5},'*')
xlabel('redundancy')
ylabel('status 4')

figure
plot(J3_spacing,sol{6},'*')
xlabel('redundancy')
ylabel('status 5')

figure
plot(J3_spacing,sol{7},'*')
xlabel('redundancy')
ylabel('status 6')

figure
plot(J3_spacing,sol{8},'*')
xlabel('redundancy')
ylabel('status 7')