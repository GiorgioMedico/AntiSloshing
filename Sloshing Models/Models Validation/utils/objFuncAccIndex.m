function F = objFuncAccIndex(alphan_)

path_array        = ["LE","TRD","TRD","Tilt_RD","Tilt_TRD","Tilt_TRD"];
dim_array         = ["3D","3D","2D","2D","3D","2D"];
motion_array      = ["LE_3D","TRD_3D","TRD_2D","Tilt_RD_2D","Tilt_TRD_3D","Tilt_TRD_2D"];

Te_array          = [  5.0  ,   2.8  ,   2.2  ,    6.5     ,    2.5      ,    2.5      ];
Delta_start_array = [  -2   ,   -1   ,   -5   ,    0       ,    -5       ,    0        ];
A_psi             = deg2rad(30);

R = 0.049;
h = 0.08;
[g, rho, m_tot, V, csi11, zita1, m1, k1, c1, alphan, l1, L1, J, k, w1] = nModeParameters(R, h, 1);
h1 = h/2 - R/csi11*tanh(csi11*h/R);

esp_k_min = -6;
esp_k_max = -1;
k_ = 10^esp_k_max;
w = 2;

EOMtype = 'NL';
gammaL = (4*h*m1)/(rho*V*R);
gammaNL1 = (h*m1*csi11^2)/(rho*V*R);
freq = 500;

N = length(motion_array);

for i = 1:N

    Te = Te_array(i);
    Delta_start_index = Delta_start_array(i);
    motion_type = motion_array(i);
    n    = freq*Te + 1;
    time = linspace(0,Te,n);
    [sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);
    time_spline = linspace(0,2*Te,2*Te*500+1);

    [axisDist, th_max, rEddx, rEddy, rEddz, phiOde, phidOde, phiddOde] = defineTraj(path_array(i), dim_array(i), n, time, sigma, sigmad, sigmadd, A_psi);
    erased_path = erase(path_array(i),["_3D","_2D"]);
    erased_path = erase(erased_path,["_LE","_RD","_TRD"]);

    if strcmp(erased_path,"Tilt")
        
        post_name = strcat('Motions_10_10_2024/full_slosh/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(A_psi)),'deg','.mat');
        load(post_name);
        tspan = [0 2*Te];
        S0 = [0 0 0 0];
        %%Phasing procedure 
        %--------------------------------------------------------------------------
        %%Linear Sloshing Model
        [tLt,sLt] = ode45(@(t,s)odeTiltMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,phiOde,phidOde,phiddOde,h,h1,g,alphan,w,'L'), tspan, S0);
        etaLt = gammaL*(sLt(:,1).^2 + sLt(:,2).^2).^0.5;
        %--------------------------------------------------------------------------
        Efps = 60; % Framerate experiment video
        end_time = 1.5*Te;
        [~, emax_index] = max(slosh_heights);
        [~, Mmax_index] = max(etaLt);
        peak_time = tLt(Mmax_index);
        E_start_index = round(emax_index - peak_time*Efps) + Delta_start_index;
        E_slosh_h_cut_1 = slosh_heights(E_start_index+1:E_start_index+(end_time*Efps));
        E_slosh_t_cut_1 = slosh_times(E_start_index+1:E_start_index+(end_time*Efps)) - slosh_times(E_start_index+1);
        %--------------------------------------------------------------------------

        %%Non-linear Sloshing Model
        [tNLt1,sNLt1] = ode45(@(t,s)odeTiltMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,phiOde,phidOde,phiddOde,h,h1,g,alphan_/R^2,w,EOMtype), tspan, S0);
        etaNLt1 = gammaNL1*(sNLt1(:,1).^2 + sNLt1(:,2).^2).^0.5;
        etaNLt1_spline = spline(tNLt1,etaNLt1,time_spline);

        %%Accuracy-index computation
        eps_n1(i) = calcola_errore_picco(1000*(etaNLt1), abs(E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000*(etaNLt1),abs(E_slosh_h_cut_1),tNLt1,E_slosh_t_cut_1,Te,1.5*Te,1);

        clear time_spline etaNLt1_spline tNLt1 sNLt1 etaNLt1

    else

        post_name = strcat('Motions_10_10_2024/full_slosh/',motion_type,'_', num2str(axisDist),'m_',num2str(Te),'s_',num2str(rad2deg(th_max)),'deg','.mat');
        load(post_name);
        tspan = [0 2*Te];
        S0 = [0 0 0 0 0 0 0];
        %%Phasing procedure 
        %--------------------------------------------------------------------------
        phidd_zero = zeros(1,n);

        %%Linear with paraboloic term 
        [tLp,sL] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k,zita1,m1,time,rEddy,-rEddx,rEddz,phidd_zero,J,g,alphan,2,'L'), tspan, S0);
        gammaL = (4*h*m1)/(rho*V*R);
        etaL = gammaL*(sL(:,1).^2 + sL(:,2).^2).^0.5;
        
        clear etaPar
        for j = 1:length(tLp)
            if tLp(j)<Te
                thetad(j) = spline(time,phidOde,tLp(j));
                etaPar(j) = R^2*thetad(j)^2/(4*g);
            else
                etaPar(j) = 0;
            end
        end
        etaLPar = etaL + etaPar';
        %--------------------------------------------------------------------------
        Efps = 60; % Framerate experiment video
        end_time = 1.5*Te;
        [~, emax_index] = max(slosh_heights);
        [~, Mmax_index] = max(etaLPar);
        peak_time = tLp(Mmax_index);
        E_start_index = round(emax_index - peak_time*Efps) + Delta_start_index;
        E_slosh_h_cut_1 = slosh_heights(E_start_index+1:E_start_index+(end_time*Efps));
        E_slosh_t_cut_1 = slosh_times(E_start_index+1:E_start_index+(end_time*Efps)) - slosh_times(E_start_index+1);
        %--------------------------------------------------------------------------

        %%Non-linear Sloshing Model
        [tNLk1,sNLk1] = ode45(@(t,s)odeSchoenMSD(t,s,k1,k_,zita1,m1,time,rEddy,-rEddx,rEddz,phiddOde,J,g,alphan_/R^2,w,EOMtype), tspan, S0);
        etaNLk1 = gammaNL1*(sNLk1(:,1).^2 + sNLk1(:,2).^2).^0.5;
        etaNLk1_spline = spline(tNLk1,etaNLk1,time_spline);

        %%Accuracy-index computation
        eps_n1(i) = calcola_errore_picco(1000*(etaNLk1), abs(E_slosh_h_cut_1), 0, 1);
        sigmaR_n1_norm(i) = errore_integrale_esaurimento(1000*(etaNLk1),abs(E_slosh_h_cut_1),tNLk1,E_slosh_t_cut_1,Te,1.5*Te,1);
 
        clear time_spline etaNLk1_spline tNLk1 sNLk1 etaNLk1
    end
    
    clear slosh_times slosh_heights E_slosh_t_cut_1 E_slosh_h_cut_1 

end


F = [eps_n1; sigmaR_n1_norm];

end




