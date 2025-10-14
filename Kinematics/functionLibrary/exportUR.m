function exportUR(qc, zimmer, lin, t, folder)
%     figure()
for i = 1:size(qc,2)
    
    if i==1 | i==2
        motion{i}.J = [t,(qc(:,i)-pi)*180/pi];
    else
        motion{i}.J = [t,(qc(:,i))*180/pi];
    end
    
%     for j = 1:size(qc,1)
%         if motion{i}.J(j,2) > 180
%             diff = motion{i}.J(j,2) - 180;
%             motion{i}.J(j,2) = -180 + diff;
%         elseif motion{i}.J(j,2) < -180
%             diff = abs(motion{i}.J(j,2) + 180);
%             motion{i}.J(j,2) = 180 - diff;
%         end
%         
%         if j > 1
%             if sign(motion{i}.J(j-1,2))~=sign(motion{i}.J(j,2)) && abs(motion{i}.J(j,2)+motion{i}.J(j-1,2))>0.2
%                 motion{i}.J(j,2) = -motion{i}.J(j,2);
%             end
%         end
%     end
    
%     subplot(2,3,i)
%     hold on
%     grid on
%     plot(t,motion{i}.J(:,2),'-o')
    
end

% dt = t(2)-t(1);
% 
% for i = 1:size(qc,2)
%     for j = 2:size(qc,1)
%         
%         motion{i}.Jd(j) = (motion{i}.J(j,2)-motion{i}.J(j-1,2))/dt;
%         
%     end
%     
%     figure(1)
%     subplot(2,3,i)
%     hold on
%     grid on
%     plot(t,motion{i}.Jd)
%     
% end
        
 


motion_zimmer = [t,zimmer];
motion_lin = [t,lin];
motion_J1 = motion{1}.J;
motion_J2 = motion{2}.J;
motion_J3 = motion{3}.J;
motion_J4 = motion{4}.J;
motion_J5 = motion{5}.J;
motion_J6 = motion{6}.J;

save([folder 'J1.tab'],'motion_J1','-ascii','-tabs')
save([folder 'J2.tab'],'motion_J2','-ascii','-tabs')
save([folder 'J3.tab'],'motion_J3','-ascii','-tabs')
save([folder 'J4.tab'],'motion_J4','-ascii','-tabs')
save([folder 'J5.tab'],'motion_J5','-ascii','-tabs')
save([folder 'J6.tab'],'motion_J6','-ascii','-tabs')
save([folder 'motion_zimmer.tab'],'motion_zimmer','-ascii','-tabs')
save([folder 'motion_lin.tab'],'motion_lin','-ascii','-tabs')

end