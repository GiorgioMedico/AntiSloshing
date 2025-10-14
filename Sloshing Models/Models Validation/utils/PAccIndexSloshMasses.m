function [Fr, Fz] = PAccIndexSloshMasses(numSloshMasses,h,R,EOMtype,N,erased_path,Te,str,k_eps,k_sigma,objFuncType)

csi1n = zeros(1,numSloshMasses);
zitan = zeros(1,numSloshMasses);
% mn = zeros(1,numSloshMasses);
% kn = zeros(1,numSloshMasses);
% cn = zeros(1,numSloshMasses);
% hn = zeros(1,numSloshMasses);
ln = zeros(1,numSloshMasses);
Ln = zeros(1,numSloshMasses);
wn = zeros(1,numSloshMasses);
dn = zeros(1,numSloshMasses);
% gammaNLn = zeros(1,numSloshMasses);
gammaPNLn = zeros(1,numSloshMasses);


for j = 1:numSloshMasses

    [g, ~, m_tot, ~, csi1n(j), zitan(j), mn(j), kn(j), cn(j), ~, ln(j), Ln(j), ~, ~, wn(j)] = nModeParameters(R, h, j);
    % hn(j) = h/2 - R/csi1n(j)*tanh(csi1n(j)*h/R);
    dn(j)= h-Ln(j);
    % gammaNLn(j) = (h*mn(j)*csi1n(j)^2)/(m_tot*R);
    gammaPNLn(j) = (h*mn(j)*csi1n(j))/(m_tot*tanh(csi1n(j)*h/R));

end

eps_n1         = zeros(1,N);
sigmaR_n1_norm = zeros(1,N);

parfor i = 1:N  % <-- Parallel for loop
% for i = 1:N  % <-- Parallel for loop

    if strcmp(erased_path(i),"Tilt")

        % etaNLt = zeros(numSloshMasses,length(str(i).time));
        % etaNLt_tot = zeros(1,length(str(i).time));
        etaNLtr = zeros(numSloshMasses,length(str(i).time));
        etaNLtr_tot = zeros(1,length(str(i).time));
        etaNLtz = zeros(numSloshMasses,length(str(i).time));
        etaNLtz_tot = zeros(1,length(str(i).time));
        etaNLttan = zeros(1,length(str(i).time));

        for j = 1:numSloshMasses

            % sNLt = RK4_odeTiltMSD_res(kn(j),zitan(j),mn(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,h,hn(j),g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
            % tNLt = str(i).time;

            sNLt = RK4_odeTiltP_res(wn(j),zitan(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,dn(j),g,EOMtype,str(i).time);
            tNLt = str(i).time;

            % etaNLt(j,:) = gammaNLn(j) * sqrt(sNLt(1,:).^2 + sNLt(2,:).^2);
            etaNLtr(j,:) = gammaPNLn(j) * sqrt(sin(sNLt(1,:)).^2 .* cos(sNLt(2,:)).^2 + sin(sNLt(2,:)).^2);
            etaNLtz(j,:) = gammaPNLn(j) * sqrt(2*(1 - cos(sNLt(2,:)).*cos(sNLt(1,:))));

            % disp(size(etaNLt))

            % etaNLt_tot = etaNLt_tot + etaNLt(j,:);
            etaNLtr_tot = etaNLtr_tot + etaNLtr(j,:);
            etaNLtz_tot = etaNLtz_tot + etaNLtz(j,:);

            if j==1
                for k = 1:length(str(i).time)
                    etaNLttan(1,k) = getSloshHeight(sNLt(1,k),sNLt(2,k),R);
                end
                eps_n1tan(i) = calcola_errore_picco(1000 * etaNLttan, abs(str(i).E_slosh_h_cut_1), 0, 1);
                sigmaR_n1tan_norm(i) = errore_integrale_esaurimento(1000 * etaNLttan, abs(str(i).E_slosh_h_cut_1), tNLt, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);
            end

        end

        % eps_n1(i) = calcola_errore_picco(1000 * etaNLt_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        % sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLt_tot, abs(str(i).E_slosh_h_cut_1), tNLt, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

        eps_n1r(i) = calcola_errore_picco(1000 * etaNLtr_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1r_norm(i) = errore_integrale_esaurimento(1000 * etaNLtr_tot, abs(str(i).E_slosh_h_cut_1), tNLt, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

        eps_n1z(i) = calcola_errore_picco(1000 * etaNLtz_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1z_norm(i) = errore_integrale_esaurimento(1000 * etaNLtz_tot, abs(str(i).E_slosh_h_cut_1), tNLt, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

        
    else

        % etaNLk = zeros(numSloshMasses,length(str(i).time));
        % etaNLk_tot = zeros(1,length(str(i).time));
        etaNLkr = zeros(numSloshMasses,length(str(i).time));
        etaNLkr_tot = zeros(1,length(str(i).time));
        etaNLkz = zeros(numSloshMasses,length(str(i).time));
        etaNLkz_tot = zeros(1,length(str(i).time));
        etaNLktan = zeros(1,length(str(i).time));

        disp("0")
        for j = 1:numSloshMasses

            % sNLk = RK4_odeSchoenMSD_res(kn(j),zitan(j),mn(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
            sNLk = RK4_odeSchoenP_res(wn(j),zitan(j),str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,g,EOMtype,str(i).time);
            tNLk = str(i).time;
            disp("1")
            % etaNLk(j,:) = gammaNLn(j) * sqrt(sNLk(1,:).^2 + sNLk(2,:).^2);
            etaNLkr(j,:) = gammaPNLn(j) * sqrt(sin(sNLk(1,:)).^2 .* cos(sNLk(2,:)).^2 + sin(sNLk(2,:)).^2);
            etaNLkz(j,:) = gammaPNLn(j) * sqrt(2*(1 - cos(sNLk(2,:)).*cos(sNLk(1,:))));

            % disp(size(etaNLk))
            disp("2")

            % etaNLk_tot = etaNLk_tot + etaNLk(j,:);
            etaNLkr_tot = etaNLkr_tot + etaNLkr(j,:);
            etaNLkz_tot = etaNLkz_tot + etaNLkz(j,:);

            disp("3")

            if j==1
                for k = 1:length(str(i).time)
                    etaNLktan(1,k) = getSloshHeight(sNLk(1,k),sNLk(2,k),R);
                end
                eps_n1tan(i) = calcola_errore_picco(1000 * etaNLktan, abs(str(i).E_slosh_h_cut_1), 0, 1);
                sigmaR_n1tan_norm(i) = errore_integrale_esaurimento(1000 * etaNLktan, abs(str(i).E_slosh_h_cut_1), tNLk, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);
            end

        end

        % eps_n1(i) = calcola_errore_picco(1000 * etaNLk_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        % sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLk_tot, abs(str(i).E_slosh_h_cut_1), tNLk, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);
        
        eps_n1r(i) = calcola_errore_picco(1000 * etaNLkr_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1r_norm(i) = errore_integrale_esaurimento(1000 * etaNLkr_tot, abs(str(i).E_slosh_h_cut_1), tNLk, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

        eps_n1z(i) = calcola_errore_picco(1000 * etaNLkz_tot, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1z_norm(i) = errore_integrale_esaurimento(1000 * etaNLkz_tot, abs(str(i).E_slosh_h_cut_1), tNLk, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

        disp("4")
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
elseif strcmp(objFuncType,'heights')
    Fr = [eps_n1r; sigmaR_n1r_norm];
    Fz = [eps_n1z; sigmaR_n1z_norm];
elseif strcmp(objFuncType,'tan')
    Fr = [eps_n1tan; sigmaR_n1tan_norm];
    Fz = [];
end

end




