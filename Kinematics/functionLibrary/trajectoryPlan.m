function [qc,zimmer,time,time_step] = trajectoryPlan(punti,robot,seqMoto,varargin)

%qc is a matrix, containing the trajectory

if isempty(varargin)
    time_step = 0.05;
else
    time_step=varargin{1};
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
time=[];
time_seg = 1;

for i=1:segmentNumber
    if strcmp(seqMoto(i),'p')
        qc_seg = p2p_JM(punti{initial_point}.J, punti{final_point}.J, time_seg-time_step, time_step);
    elseif strcmp(seqMoto(i),'l')
        [qc_seg,~] = lin_JM(punti{initial_point}.T,punti{final_point}.T,time_seg-time_step,time_step,robot,punti{initial_point}.ridondanza,punti{initial_point}.status);
    else
        error('tipo di moto sconosciuto');
    end
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
    time=[time; (0:time_step:time_seg-time_step)'+(i-1)*time_seg];
end


if isempty(zimmer)
    zimmer = zeros(length(qc),1);
end

%  disp(['DURATA SIMULAZIONE: ', num2str(max(time)), ' sec'])