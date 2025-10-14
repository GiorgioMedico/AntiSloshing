% Creata da Claudio Mazzotti

function [qc,zimmer,t] = unifica_ed_esporta(qc,zimmer,time,time_step)

QC = qc;
qc = [];
t = [];
ZIMMER = zimmer;
zimmer = [];

dim_segmenti = [];

for ii = 1:1:size(QC,2)
    
    dim_segmenti(ii) = size(QC{ii},1);
    if isempty(QC{ii})
        disp(['MOTO NON ESEGUIBILE'])
        disp(['ESPORTAZIONE FALLITA'])
        return
    elseif isnan(QC{ii})
        disp(['MOTO NON ESEGUIBILE'])
        disp(['ESPORTAZIONE FALLITA'])
        return
    else
        
    end
end

total_time = 0;
time_segmenti = [];
for i = 1:1:size(time,2)
total_time = time{i}+total_time;
time_segmenti(i) = (time{i}/time_step)+1;
end
t = [];
t = [0:time_step:total_time]';

% verifica che il moto sia stato interamente pianificato, ovvero che i
% segmenti abbiano la stessa lunghezza --> ERRORE
% verifica che 
% dim_segmenti = [dim_segmenti size(t,1)];
check_segmenti = (dim_segmenti-time_segmenti)==0;
kk = [];
for kk = 1:1:(size(check_segmenti,2)-1)
    if isequal(check_segmenti(kk),check_segmenti(kk+1))
    else
        clc
        disp(['MOTO NON ESEGUIBILE'])
        disp(['ESPORTAZIONE FALLITA'])
        t = [];
        qc = [];
        zimmer = [];
        return
    end
end


for i = 1:1:size(QC,2)
    if i<size(QC,2)
    qc = [qc;QC{i}(1:end-1,:)];
    elseif i==size(QC,2)
    qc = [qc;QC{i}];
    end
end

if isempty(ZIMMER)
zimmer = zeros(length(qc),1);
else
for i = 1:1:size(ZIMMER,2)
    if i<size(ZIMMER,2)
    zimmer = [zimmer;ZIMMER{i}(1:end-1)];
    elseif i==size(ZIMMER,2)
    zimmer = [zimmer;ZIMMER{i}(1:end)];
    end
end
end

disp(['DURATA SIMULAZIONE: ', num2str(total_time), ' sec'])

%% export delle leggi di moto per il CAD
motion_J1 = [t,qc(:,1)*180/pi];
motion_J2 = [t,qc(:,2)*180/pi];
motion_J3 = [t,qc(:,3)*180/pi];
motion_J4 = [t,qc(:,4)*180/pi];
motion_J5 = [t,qc(:,5)*180/pi];
motion_J6 = [t,qc(:,6)*180/pi];
motion_J7 = [t,qc(:,7)*180/pi];
motion_zimmer = [t,zimmer];

save('J1.tab','motion_J1','-ascii','-tabs')
save('J2.tab','motion_J2','-ascii','-tabs')
save('J3.tab','motion_J3','-ascii','-tabs')
save('J4.tab','motion_J4','-ascii','-tabs')
save('J5.tab','motion_J5','-ascii','-tabs')
save('J6.tab','motion_J6','-ascii','-tabs')
save('J7.tab','motion_J7','-ascii','-tabs')
save('motion_zimmer.tab','motion_zimmer','-ascii','-tabs')