function [qc,zimmer,lin,time,time_step] = trajectoryPlan_UR(punti,robot,seqMoto,varargin,timeargin)

%qc is a matrix, containing the trajectory

if isempty(varargin)
    time_step = 0.05;
else
    time_step=varargin(1);
end


%% segmenti

segmentNumber= size(punti,2)-1;
initial_point=1;
final_point = 2;

if length(seqMoto)~=segmentNumber
    error('errore nella definizione della sequenza di tipi di moto');
end
qc=[];
zimmer = [];
lin = [];
time=[];
time_end = 0;


for i=1:segmentNumber
    
    if isempty(timeargin)
        time_seg = 1;
    else
        time_seg = timeargin(i);
    end
    
    if strcmp(seqMoto(i),'p')
        status = punti{initial_point}.status;
        qc_seg = p2p_JM_UR(punti{initial_point}, punti{final_point}, time_seg-time_step, time_step, status);
%         lin_seg = zimmer_JM(punti{initial_point}.h,  punti{final_point}.h, time, time_step);
    elseif strcmp(seqMoto(i),'l')
        status = punti{initial_point}.status;
        [qc_seg,~] = lin_JM_UR(punti{initial_point}.T,punti{final_point}.T,time_seg-time_step,time_step,robot, status);
%         lin_seg = zimmer_JM(punti{initial_point}.h,  punti{final_point}.h, time_seg, time_step);
    else
        error('tipo di moto sconosciuto');
    end
%     lin_seg = zimmer_JM(punti{initial_point}.h,  punti{final_point}.h, time_seg-time_step, time_step);
    initial_point=final_point;
    final_point = final_point+1; 
    
    if isempty(qc_seg)
        disp('MOTO NON ESEGUIBILE')
        disp('ESPORTAZIONE FALLITA')
        return
    elseif isnan(qc_seg)
        disp('MOTO NON ESEGUIBILE')
        disp('ESPORTAZIONE FALLITA')
        return
    end
    
    qc=[qc; qc_seg;];
%     lin = [lin; lin_seg];
if isempty(varargin)
    time=[time; (0:time_step:time_seg-time_step)'+(i-1)*time_seg];
else
    time=[time; (0:time_step:time_seg-time_step)' + time_end];
    time_end = time_end+time_seg;
end
    
end

% Aggiunta ultimo tratto
qc = [qc; punti{size(punti,2)}.J];
% lin = [lin; punti{size(punti,2)}.h];
% time=[time; segmentNumber];
    if isempty(timeargin)
        time=[time; segmentNumber];
    else
        time=[time; sum(timeargin)];
    end


if isempty(zimmer)
    zimmer = zeros(length(qc),1);
end

% for i = 2:size(qc,1)
%     for j = 1:size(qc,2)
%         diff = qc(i,j) - qc(i-1,j);
%         absdiff = abs(qc(i,j)) - abs(qc(i-1,j));
%         if abs(absdiff) < 1e-01 && sign(qc(i,j)) ~=  sign(qc(i-1,j))
%             qc(i,j) = qc(i-1,j) + sign(qc(i-1,j))*absdiff;
%         end         
%          
%     end
% end

end
%  disp(['DURATA SIMULAZIONE: ', num2str(max(time)), ' sec'])