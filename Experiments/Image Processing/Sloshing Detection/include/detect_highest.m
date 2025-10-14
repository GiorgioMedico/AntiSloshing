function [h_max, ind_h_max] = detect_highest(BW,det_ind1,det_ind2,h0_det_px,type)

close all

k = 0;
for j = det_ind1:det_ind2
    
    k = k+1;
    ind = find(BW(h0_det_px:end,j)==type);
    max_height_px = h0_det_px + min(ind);
    
    if isempty(ind)
        height(k) = 1080;
    else 
        height(k) = max_height_px;    
    end
    
end

% h_max = min(height);
[h_max, ind_h_max] = min(height); 
ind_h_max = ind_h_max + det_ind1;

figure()
hold on
grid minor
imshow(BW)
line([ind_h_max+det_ind1 ind_h_max+det_ind1],[h_max-20 h_max+20],'Color','green','LineWidth',2.5)
line([ind_h_max+det_ind1-20 ind_h_max+det_ind1+20],[h_max h_max],'Color','green','LineWidth',2.5)
pause(0.1)
line([ind_h_max ind_h_max],[h_max+10 h_max-10],'Color','green','LineWidth',2.5)
end