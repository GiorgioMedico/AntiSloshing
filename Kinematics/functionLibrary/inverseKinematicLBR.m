% Creata da Claudio Mazzotti
% Aggiornata da Francesco Meoni

% Goal Point [1x6] = mm e gradi
% Goal Point [4x4] = mm e coseni direttori
% KUKA_R = ridondanza in gradi
% status tra 0 e 7

%A--- [robot_solution]         = inverseKinematicLBR(robot,GoalPoint,KUKA_R,status_richiesto)
%        richiesto uno status specifico, restituisce la soluzione unica ROBOT_SOLUTION.

%B--- [robot_solutions,status] = inverseKinematicLBR(robot,GoalPoint,KUKA_R)
%       status non richiesto, restituisce una matrice ROBOT_SOLUTIONS
%       che contiene una soluzione in ogni riga. Il vettore STATUS contiene
%       i corrispondenti status possibili

function [robot_solution,varargout] = inverseKinematicLBR(robot,GoalPoint,KUKA_R,varargin)

if ~isempty(varargin)
    status_richiesto=varargin{1};
end

%% assegno la posa_goal
L1 = robot.links(1,1).d;
L2 = robot.links(1,3).d;
L3 = robot.links(1,5).d;
L4 = robot.links(1,7).d;

if all(size(GoalPoint)==[1 6])
    % valori scritti nel sistema World di KUKA (già ruotato di 45°!)
    KUKA_X = GoalPoint(1);%mm
    KUKA_Y = GoalPoint(2);%mm
    KUKA_Z = GoalPoint(3);%mm
    KUKA_A = GoalPoint(4);%gradi
    KUKA_B = GoalPoint(5);%gradi
    KUKA_C = GoalPoint(6);%gradi
    T_W45_TCP = transl(KUKA_X*10^-3,KUKA_Y*10^-3,KUKA_Z*10^-3)*trotz(KUKA_A,'deg')*troty(KUKA_B,'deg')*trotx(KUKA_C,'deg');
    
elseif all(size(GoalPoint)==[4 4])
    GoalPoint(1:3,4) = GoalPoint(1:3,4)*10^-3;
    T_W45_TCP = GoalPoint;
end

% Ttool = tool;%sistema tool rispetto alla flangia
% Ttool(1:3,4) = Ttool(1:3,4)*10^-3;
angolo_base_maxima = 45;
angolo_base_euroc = 0;
T_W0_TCP = inv(transl(0,0,L1))*inv(troty(angolo_base_maxima,'deg'))*T_W45_TCP;%*inv(Ttool);

%% assegno q3
q3 = KUKA_R*pi/180;
robot_solutions = zeros(8,7);
robot_solutions(1:8,3) = q3;

%% calcolo q4
q4 = calcolo_q4(robot,T_W0_TCP);

robot_solutions(1:4,4) = q4(1);
robot_solutions(5:8,4) = q4(2);

%% calcolo q2
q2_1 = calcolo_q2(robot,T_W0_TCP,q3,q4(1));
robot_solutions(1:2,2) = q2_1(1);
robot_solutions(3:4,2) = q2_1(2);

q2_2 = calcolo_q2(robot,T_W0_TCP,q3,q4(2));
robot_solutions(5:6,2) = q2_2(1);
robot_solutions(7:8,2) = q2_2(2);

%% calcolo q1
q1_1 = calcolo_q1(robot,T_W0_TCP,q2_1(1),q3,q4(1));
q1_2 = calcolo_q1(robot,T_W0_TCP,q2_1(2),q3,q4(1));
q1_3 = calcolo_q1(robot,T_W0_TCP,q2_2(1),q3,q4(2));
q1_4 = calcolo_q1(robot,T_W0_TCP,q2_2(2),q3,q4(2));

robot_solutions(1:2,1) = q1_1;
robot_solutions(3:4,1) = q1_2;
robot_solutions(5:6,1) = q1_3;
robot_solutions(7:8,1) = q1_4;

%% calcolo q5-q6-q7
solution=1;
[q5,q6,q7] = calcolo_q5q6q7(robot,T_W0_TCP,robot_solutions(solution,1),robot_solutions(solution,2),robot_solutions(solution,3),robot_solutions(solution,4));
robot_solutions(solution,5) = q5(1);
robot_solutions(solution,6) = q6(1);
robot_solutions(solution,7) = q7(1);
robot_solutions(solution+1,5) = q5(2);
robot_solutions(solution+1,6) = q6(2);
robot_solutions(solution+1,7) = q7(2);

solution=3;
[q5,q6,q7] = calcolo_q5q6q7(robot,T_W0_TCP,robot_solutions(solution,1),robot_solutions(solution,2),robot_solutions(solution,3),robot_solutions(solution,4));
robot_solutions(solution,5) = q5(1);
robot_solutions(solution,6) = q6(1);
robot_solutions(solution,7) = q7(1);
robot_solutions(solution+1,5) = q5(2);
robot_solutions(solution+1,6) = q6(2);
robot_solutions(solution+1,7) = q7(2);

solution=5;
[q5,q6,q7] = calcolo_q5q6q7(robot,T_W0_TCP,robot_solutions(solution,1),robot_solutions(solution,2),robot_solutions(solution,3),robot_solutions(solution,4));
robot_solutions(solution,5) = q5(1);
robot_solutions(solution,6) = q6(1);
robot_solutions(solution,7) = q7(1);
robot_solutions(solution+1,5) = q5(2);
robot_solutions(solution+1,6) = q6(2);
robot_solutions(solution+1,7) = q7(2);

solution=7;
[q5,q6,q7] = calcolo_q5q6q7(robot,T_W0_TCP,robot_solutions(solution,1),robot_solutions(solution,2),robot_solutions(solution,3),robot_solutions(solution,4));
robot_solutions(solution,5) = q5(1);
robot_solutions(solution,6) = q6(1);
robot_solutions(solution,7) = q7(1);
robot_solutions(solution+1,5) = q5(2);
robot_solutions(solution+1,6) = q6(2);
robot_solutions(solution+1,7) = q7(2);

%% studio fattibilità meccanica delle soluzioni
robot_reachable_solutions = zeros(8,7);

for i = 1:8
    check1 = (robot_solutions(i,:)' < robot.qlim(:,1))==0;
    check2 = (robot_solutions(i,:)' > robot.qlim(:,2))==0;
    
    if all(check1) && all(check2)
        robot_reachable_solutions(i,:) = robot_solutions(i,:);
    else
        robot_reachable_solutions(i,:) = NaN;
        % disp(['soluzione non raggiungile: ', num2str(i)]),
    end
    
end

for i=1:8
    if all(isfinite(robot_reachable_solutions(i,:)))==0
        robot_reachable_solutions(i,:) = NaN;
    end
end

%% status conformi con KUKA

for i = 1:8
    
    if  isnan(robot_reachable_solutions(i,1));
        status(i) = NaN;
        continue
    end
    
    T_w45_tcp = robot.fkine(robot_reachable_solutions(i,:));
    
    T_wA1_w0 = inv(trotz(robot_reachable_solutions(i,1)));
    T_w0_w45 = inv(robot.base);
    T_wA1_tcp = T_wA1_w0*T_w0_w45*T_w45_tcp*transl(0,0,-L4);
    
    x_A1 = T_wA1_tcp(1,4);
    if x_A1>=0
        bit_0 = 0;
    else
        bit_0 = 1;
    end
    
    if robot_reachable_solutions(i,4)>=0
        bit_1 = 0;
    else
        bit_1 = 1;
    end
    
    %%%%%%%%%%%%%%%%%mettere 6!
    if robot_reachable_solutions(i,6)>0
        bit_2 = 0;
    else
        bit_2 = 1;
    end
    stat_bit(i,:)=[bit_0,bit_1,bit_2];
%     status(i) = bi2de([bit_0,bit_1,bit_2]);
    status(i) = bin2dec(num2str([bit_2,bit_1,bit_0]));
end

%% guarda status possibili
% [B,I] = sort(status);
status_possibili = status(find(isnan(status)==0));
% disp(['status richiesto: ', num2str(status_richiesto)]);

if isempty(varargin)
    %se NON è richiesto uno stato specifico
    robot_solution = robot_reachable_solutions(~isnan(status),:);

    if isempty(robot_solution)
%         disp('No available solutions with given redundancy');
    else
%         disp(['status possibili: ', num2str(status_possibili)]);
    end
    varargout{1}=status_possibili;
else
    %se è richiesto uno stato specifico
    if isempty(find(status==status_richiesto,1))
       %disp(['status NON raggiungibile']);
        robot_solution = ones(1,7)*NaN;
    else
%       disp(['status raggiungibile']);
        find_status = find(status==status_richiesto);
        robot_solution = robot_reachable_solutions(find_status,:);
    end
end

