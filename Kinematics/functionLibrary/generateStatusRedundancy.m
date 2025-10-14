function [trajStatRed]=generateStatusRedundancy(punti,radius,robot,graph)

%generates possible combination of status and redundancy, checking the
%existence of TCP (thus kmr) position in a radius (xy plane TCP)

%radius= mm
%queste 3 variabili sono in gradi di ridondanza. red sta per redundacy
minRedRange=6; %not consider interval which length is <minRedRange; 
redSpacing=40; %spacing between two trajectory;
redEdge=2; %do not consider a trajectory if it is near the edge (dist<redEdge);
%It should be redEdge<minRedRange/2;

%% ANALISI INTORNO
%considerare anche un'intorno di posizioni del TCP (quindi del kmr)
Npose=size(punti,2);

%aggiungi punti
for p=1:Npose
    punti{p+  Npose}.T= punti{p}.T*transl(0, radius,0);
    punti{p+2*Npose}.T= punti{p}.T*transl(0,-radius,0);
    punti{p+3*Npose}.T= punti{p}.T*transl( radius,0,0);
    punti{p+4*Npose}.T= punti{p}.T*transl(-radius,0,0);
end
NposeExt=size(punti,2);
resultMatrix=zeros(8,341,Npose);
for p=1:NposeExt
    resultMatrix(:,:,p)=feasibilityAnalysis(punti{p}.T ,robot);
end
[RedIntStr]=redundancyIntervalAnalysis(resultMatrix,robot);

if graph
    plotFeasibilityIndex(resultMatrix,robot);
end

trajCount=0; %initializing trajectory count
for j=1:8
    for k=1:RedIntStr(j).NoI
        value=RedIntStr(j).value(k);
        delim=RedIntStr(j).delimiters(k,:);

                if value>=minRedRange
                    meanRed=mean(delim);
                    chosenRed=[flip(meanRed:-redSpacing:delim(1))    meanRed+redSpacing:redSpacing:delim(2)];

                    for cr=1:length(chosenRed)
                        if chosenRed(cr)>delim(1)+redEdge && chosenRed(cr)<delim(2)-redEdge
                            trajCount=trajCount+1;
                            trajStatRed(trajCount,1)=j-1;
                            trajStatRed(trajCount,2)=chosenRed(cr);
                            if graph
                                subplot(2,4,trajStatRed(trajCount,1)+1)
                                hold on
                                plot(trajStatRed(trajCount,2),1.2,'k+','LineWidth',3.5,'MarkerSize',15);
                                text(trajStatRed(trajCount,2),1.15,num2str(trajStatRed(trajCount,2)) ) ;
                            end
                        end
                    end
                end 
    end
end
if trajCount==0
    trajStatRed=[];
    (' Nessuna traiettoria trovata che soddisfa i parametri richiesti');
end



