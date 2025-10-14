function h0_px = detect_h0(BW,det_ind1,det_ind2,h0_det_px,nsteps,type)

steps = round(linspace(det_ind1,det_ind2,nsteps+1));
% nsteps = length(steps);
k = 0;
for j = steps
    
    k = k+1;
    ind = find(BW(h0_det_px:end,j)==type);
    max_height_px = h0_det_px + min(ind);
    
    height(k) = max_height_px;    
    
end

h0_px = round(sum(height)/(nsteps+1));

end