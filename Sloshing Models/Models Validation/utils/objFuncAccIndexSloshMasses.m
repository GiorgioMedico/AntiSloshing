function F = objFuncAccIndexSloshMasses(alphan_,w,numSloshMasses,h,R,EOMtype,N,erased_path,Te,str,k_eps,k_sigma,objFuncType)

csi1n = zeros(1,numSloshMasses);
zitan = zeros(1,numSloshMasses);
mn = zeros(1,numSloshMasses);
kn = zeros(1,numSloshMasses);
cn = zeros(1,numSloshMasses);
hn = zeros(1,numSloshMasses);
gammaNLn = zeros(1,numSloshMasses);

for j = 1:numSloshMasses

    [g, ~, m_tot, ~, csi1n(j), zitan(j), mn(j), kn(j), cn(j), ~, ~, ~, ~, ~, ~] = nModeParameters(R, h, j);
    hn(j) = h/2 - R/csi1n(j)*tanh(csi1n(j)*h/R);
    gammaNLn(j) = (h*mn(j)*csi1n(j)^2)/(m_tot*R);

end

eps_n1         = zeros(1,N);
sigmaR_n1_norm = zeros(1,N);

parfor i = 1:N  % <-- Parallel for loop
% for i = 1:N  % <-- Parallel for loop

    if strcmp(erased_path(i),"Tilt")

        etaNLt = zeros(numSloshMasses,length(str(i).time));
        etaNLt_tot = zeros(1,length(str(i).time));

        for j = 1:numSloshMasses

            sNLt = RK4_odeTiltMSD_res(kn(j),zitan(j),mn(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,h,hn(j),g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
            tNLt = str(i).time;
            etaNLt(j,:) = gammaNLn(j) * sqrt(sNLt(1,:).^2 + sNLt(2,:).^2);

            etaNLt_tot = etaNLt_tot + etaNLt(j,:);

        end

        eps_n1(i) = calcola_errore_picco(1000 * etaNLt_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLt_tot, abs(str(i).E_slosh_h_cut_1), tNLt, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);


    else

        etaNLk = zeros(numSloshMasses,length(str(i).time));
        etaNLk_tot = zeros(1,length(str(i).time));

        for j = 1:numSloshMasses

            sNLk = RK4_odeSchoenMSD_res(kn(j),zitan(j),mn(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
            tNLk = str(i).time;
            etaNLk(j,:) = gammaNLn(j) * sqrt(sNLk(1,:).^2 + sNLk(2,:).^2);

            etaNLk_tot = etaNLk_tot + etaNLk(j,:);

        end

        eps_n1(i) = calcola_errore_picco(1000 * etaNLk_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLk_tot, abs(str(i).E_slosh_h_cut_1), tNLk, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

    end
end


if strcmp(objFuncType,'abs_value')
    F = k_eps*sum(abs(eps_n1))/N + k_sigma*sum(abs(sigmaR_n1_norm))/N;
elseif strcmp(objFuncType,'wSign_value')
    F = k_eps*sum(eps_n1)/N + k_sigma*sum(sigmaR_n1_norm)/N;
elseif strcmp(objFuncType,'vect_value')
    F = [eps_n1; sigmaR_n1_norm];
elseif strcmp(objFuncType,'all_value')
    F = [eps_n1,         k_eps*sum(abs(eps_n1))/N + k_sigma*sum(abs(sigmaR_n1_norm))/N;
         sigmaR_n1_norm, k_eps*sum(eps_n1)/N + k_sigma*sum(sigmaR_n1_norm)/N         ];
end

end




