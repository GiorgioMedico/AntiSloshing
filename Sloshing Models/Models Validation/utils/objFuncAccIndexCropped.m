function F = objFuncAccIndexCropped(alphan_,w,m1,k1,zita1,h1,J,g,k_,h,R,EOMtype,N,erased_path,Te,gammaNL1,str,k_eps,k_sigma,objFuncType)

% eps_n1         = zeros(1,N);
% sigmaR_n1_norm = zeros(1,N);
% 
% for i = 1:N
% 
%     if strcmp(erased_path(i),"Tilt")
% 
%         tspan = [0 2*Te(i)];
%         S0 = [0 0 0 0];
%         %%Non-linear Sloshing Model
%         [tNLt1,sNLt1] = ode45(@(t,s)odeTiltMSD(t,s,k1,k_,zita1,m1,str(i).time,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,h,h1,g,alphan_/R^(2*w-2),w,EOMtype), tspan, S0);
%         etaNLt1 = gammaNL1*(sNLt1(:,1).^2 + sNLt1(:,2).^2).^0.5;
%         % etaNLt1_spline = spline(tNLt1,etaNLt1,time_spline);
% 
%         %%Accuracy-index computation
%         eps_n1(i) = calcola_errore_picco(1000*(etaNLt1), abs(str(i).E_slosh_h_cut_1), 0, 1);
%         sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000*(etaNLt1),abs(str(i).E_slosh_h_cut_1),tNLt1,str(i).E_slosh_t_cut_1,Te(i),1.5*Te(i),1);
% 
%         clear time_spline etaNLt1_spline tNLt1 sNLt1 etaNLt1
% 
%     else
% 
%         tspan = [0 2*Te(i)];
%         S0 = [0 0 0 0 0 0 0];
%         %%Non-linear Sloshing Model
%         [tNLk1,sNLk1] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k_,zita1,m1,str(i).time,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiddOde,J,g,alphan_/R^(2*w-2),w,EOMtype), tspan, S0);
%         etaNLk1 = gammaNL1*(sNLk1(:,1).^2 + sNLk1(:,2).^2).^0.5;
%         % etaNLk1_spline = spline(tNLk1,etaNLk1,time_spline);
% 
%         %%Accuracy-index computation
%         eps_n1(i) = calcola_errore_picco(1000*(etaNLk1), abs(str(i).E_slosh_h_cut_1), 0, 1);
%         sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000*(etaNLk1),abs(str(i).E_slosh_h_cut_1),tNLk1,str(i).E_slosh_t_cut_1,Te(i),1.5*Te(i),1);
% 
%         clear time_spline etaNLk1_spline tNLk1 sNLk1 etaNLk1
% 
%     end
% 
% end

eps_n1         = zeros(1,N);
sigmaR_n1_norm = zeros(1,N);

parfor i = 1:N  % <-- Parallel for loop
% for i = 1:N  % <-- Parallel for loop

    if strcmp(erased_path(i),"Tilt")

        % tspan = [0 2*Te(i)];
        % % tspan = 0:1/500:2*Te(i);
        % S0 = [0 0 0 0];
        % [tNLt1,sNLt1] = ode45(@(t,s)odeTiltMSD(t,s,k1,k_,zita1,m1,str(i).time,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,h,h1,g,alphan_/R^(2*w-2),w,EOMtype), tspan, S0);
        % etaNLt1 = gammaNL1 * sqrt(sNLt1(:,1).^2 + sNLt1(:,2).^2);

        sNLt1 = RK4_odeTiltMSD_res(k1,zita1,m1,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,h,h1,g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
        tNLt1 = str(i).time;
        etaNLt1 = gammaNL1 * sqrt(sNLt1(1,:).^2 + sNLt1(2,:).^2);

        eps_n1(i) = calcola_errore_picco(1000 * etaNLt1, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLt1, abs(str(i).E_slosh_h_cut_1), tNLt1, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);


    else

        % tspan = [0 2*Te(i)];
        % % tspan = 0:1/500:2*Te(i);
        % S0 = [0 0 0 0 0 0 0];
        % [tNLk1,sNLk1] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k_,zita1,m1,str(i).time,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiddOde,J,g,alphan_/R^(2*w-2),w,EOMtype), tspan, S0);
        % etaNLk1 = gammaNL1 * sqrt(sNLk1(:,1).^2 + sNLk1(:,2).^2);

        sNLk1 = RK4_odeSchoenMSD_res(k1,zita1,m1,str(i).rEddy,-str(i).rEddx,str(i).rEddz,str(i).phiOde,str(i).phidOde,str(i).phiddOde,g,alphan_/R^(2*w-2),w,EOMtype,str(i).time);
        tNLk1 = str(i).time;
        etaNLk1 = gammaNL1 * sqrt(sNLk1(1,:).^2 + sNLk1(2,:).^2);

        eps_n1(i) = calcola_errore_picco(1000 * etaNLk1, abs(str(i).E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000 * etaNLk1, abs(str(i).E_slosh_h_cut_1), tNLk1, str(i).E_slosh_t_cut_1, Te(i), 1.5 * Te(i), 1);

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




