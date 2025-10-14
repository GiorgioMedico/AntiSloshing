% Creata da Claudio Mazzotti, modificata da Francesco Meoni (vedi
% unifica_e_esporta)

function export(qc,zimmer,t,folder)
%% export delle leggi di moto per il CAD
motion_J1 = [t,qc(:,1)*180/pi];
motion_J2 = [t,qc(:,2)*180/pi];
motion_J3 = [t,qc(:,3)*180/pi];
motion_J4 = [t,qc(:,4)*180/pi];
motion_J5 = [t,qc(:,5)*180/pi];
motion_J6 = [t,qc(:,6)*180/pi];
motion_J7 = [t,qc(:,7)*180/pi];
motion_zimmer = [t,zimmer];
%

save([folder 'J1.tab'],'motion_J1','-ascii','-tabs')
save([folder 'J2.tab'],'motion_J2','-ascii','-tabs')
save([folder 'J3.tab'],'motion_J3','-ascii','-tabs')
save([folder 'J4.tab'],'motion_J4','-ascii','-tabs')
save([folder 'J5.tab'],'motion_J5','-ascii','-tabs')
save([folder 'J6.tab'],'motion_J6','-ascii','-tabs')
save([folder 'J7.tab'],'motion_J7','-ascii','-tabs')
save([folder 'motion_zimmer.tab'],'motion_zimmer','-ascii','-tabs')


% save('J1.tab','motion_J1','-ascii','-tabs')
% save('J2.tab','motion_J2','-ascii','-tabs')
% save('J3.tab','motion_J3','-ascii','-tabs')
% save('J4.tab','motion_J4','-ascii','-tabs')
% save('J5.tab','motion_J5','-ascii','-tabs')
% save('J6.tab','motion_J6','-ascii','-tabs')
% save('J7.tab','motion_J7','-ascii','-tabs')
% save('motion_zimmer.tab','motion_zimmer','-ascii','-tabs')