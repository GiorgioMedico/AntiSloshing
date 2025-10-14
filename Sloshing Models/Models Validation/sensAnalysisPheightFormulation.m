%--------------------------------------------------------------------------
% Analysis on height formulation influence
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it 
%    Date:       May 2025
%--------------------------------------------------------------------------

clear all
close all
clc

% ws_path = fullfile('..', '..','..');
% kin_path = fullfile(ws_path,"Kinematics");
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
% 
% addpath(genpath(kin_path));
% addpath(genpath(odes_path));
% addpath("utils")


%% Begin
% delete(gcp('nocreate'));
% parpool('local', 4);
fig_name = strcat('Motions_10_10_2024/IndexTables/');
save_fig = 0;
save_csv = 0;

tic

% path_array        = ["LE","TRD","TRD","Tilt_RD","Tilt_TRD","Tilt_TRD"];
% dim_array         = ["3D","3D","2D","2D","3D","2D"];
% motion_array      = ["LE_3D","TRD_3D","TRD_2D","Tilt_RD_2D","Tilt_TRD_3D","Tilt_TRD_2D"];
% 
% Te_array          = [  5.0  ,   2.8  ,   2.2  ,    6.5     ,    2.5      ,    2.5      ];
% Delta_start_array = [  -2   ,   -1   ,   -5   ,    0       ,    -5       ,    0        ];
% path_array        = ["LE","RD","RD","Tilt_LE","Tilt_LE","Tilt_RD"];
% dim_array         = ["2D","3D","2D","3D","2D","3D"];
% motion_array      = ["LE_2D","RD_3D","RD_2D","Tilt_LE_3D","Tilt_LE_2D","Tilt_RD_3D"];
% 
% Te_array          = [  5.0  ,   6.3  ,   6.3  ,    6.0     ,    6.0      ,    6.5      ];
% Delta_start_array = [  30   ,    0   ,    0   ,    0       ,     0       ,     40        ];
path_array        = ["LE","LE","RD","RD","TRD","TRD","Tilt_LE","Tilt_LE","Tilt_RD","Tilt_RD","Tilt_TRD","Tilt_TRD"];
dim_array         = ["2D","3D","2D","3D","2D","3D","2D","3D","2D","3D","2D","3D"];
motion_array      = ["LE_2D","LE_3D","RD_2D","RD_3D","TRD_2D","TRD_3D","Tilt_LE_2D","Tilt_LE_3D","Tilt_RD_2D","Tilt_RD_3D","Tilt_TRD_2D","Tilt_TRD_3D"];
motion_array_new  = ["Se2","Se3","Sc2","Sc3","Sg2","Sg3","Te2","Te3","Tc2","Tc3","Tg2","Tg3"];

Te_array          = [  5.0  ,  5.0  ,  6.3  ,  6.3  ,  2.2  ,  2.8...
                       6.0  ,  6.0  ,  6.5  ,  6.5  ,  2.5  ,  2.5];
Delta_start_array = [  30   ,  -2   ,   0   ,   0   ,  -5   ,  -1 ...
                        0   ,   0   ,   0   ,  40   ,   0   ,  -5];

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

%% Import experimental data

for i = 1:N

    Te = Te_array(i);
    Delta_start_index = Delta_start_array(i);
    motion_type = motion_array(i);
    n    = freq*Te + 1;
    time = linspace(0,Te,n);
    [sigma,sigmad,sigmadd] = motion_law(0,1,0,0,time);

    timeR = linspace(Te+1/freq,2*Te,freq*Te);

    [axisDist, th_max, rEddx, rEddy, rEddz, phiOde, phidOde, phiddOde] = defineTraj(path_array(i), dim_array(i), n, time, sigma, sigmad, sigmadd, A_psi);
    erased_path(i) = erase(path_array(i),["_3D","_2D"]);
    erased_path(i) = erase(erased_path(i),["_LE","_RD","_TRD"]);

    str(i).motion_type = motion_array(i);
    str(i).time  = [time, timeR];
    str(i).rEddx = [rEddx, zeros(1,freq*Te)];
    str(i).rEddy = [rEddy, zeros(1,freq*Te)];
    str(i).rEddz = [rEddz, zeros(1,freq*Te)];
    str(i).phiOde   = [phiOde, phiOde(end)*ones(1,freq*Te)];
    str(i).phidOde  = [phidOde, zeros(1,freq*Te)];
    str(i).phiddOde = [phiddOde, zeros(1,freq*Te)];

    if strcmp(erased_path(i),"Tilt")
        
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

    end
    
    str(i).E_slosh_t_cut_1 = E_slosh_t_cut_1;
    str(i).E_slosh_h_cut_1 = E_slosh_h_cut_1;

    clear slosh_times slosh_heights E_slosh_t_cut_1 E_slosh_h_cut_1 

end
toc


%% Study on f_obj 
% 
% alphan_range = linspace(0,1.5,46);
% w_range = [1, 2, 3];
% % w_range = 2;
% numSloshMasses = 1
% 
% tic
% for i = 1:length(alphan_range)
% 
%     for j = 1:length(w_range)
% 
%         % index = objFuncAccIndexCropped(alphan_range(i),w_range(j),m1,k1,zita1,h1,J,g,k_,h,R,EOMtype,N,erased_path,Te_array,gammaNL1,str,1,0,'all_value');
%         % indexSigma = objFuncAccIndexCropped(alphan_range(i),w_range(j),m1,k1,zita1,h1,J,g,k_,h,R,EOMtype,N,erased_path,Te_array,gammaNL1,str,0,1,'abs_value');
%         index = objFuncAccIndexSloshMasses(alphan_range(i),w_range(j),numSloshMasses,h,R,EOMtype,N,erased_path,Te_array,str,1,0,'all_value');  
%         indexSigma = objFuncAccIndexSloshMasses(alphan_range(i),w_range(j),numSloshMasses,h,R,EOMtype,N,erased_path,Te_array,str,0,1,'abs_value');  
% 
%         f_obj_eps(i,:,j)   = index(1,:);
%         f_obj_sigma(i,:,j) = [index(2,1:N), indexSigma];
% 
%         absIndexEps(i,j) = sum(abs(f_obj_eps(i,1:N,j)))/N;
%         absIndexSigma(i,j) = sum(abs(f_obj_sigma(i,1:N,j)))/N;
% 
%     end
% 
% end
% toc

%% Study on f_obj 
numSloshMasses = 3

tic
for i = 1:numSloshMasses

        [indexR, indexZ] = PAccIndexSloshMasses(i,h,R,EOMtype,N,erased_path,Te_array,str,1,0,'heights');  

        epsR_all(i,:) = indexR(1,:);
        epsZ_all(i,:) = indexZ(1,:);

        sigmaR_all(i,:) = indexR(2,:);
        sigmaZ_all(i,:) = indexZ(2,:);

end
toc

eps = [epsR_all; epsZ_all];
sigma = [sigmaR_all; sigmaZ_all];

[index,~] = PAccIndexSloshMasses(1,h,R,EOMtype,N,erased_path,Te_array,str,1,0,'tan');  
eps = [eps; index(1,:)];
sigma = [sigma;index(2,:)];


%% Table

% T = table([xSol; alpha_sota', w_sota'],[eps_; eps_sota],[sigmaR_; sigmaR_sota],[k_eps; zeros(length(w_sota),1)],[k_sigma; zeros(length(w_sota),1)],[fmin; zeros(length(w_sota),1)]);
% colNames = {'\alpha_n','w','\epsilon','\sigma_r','k_{\epsilon}','k_{\sigma}','f_{obj}'};
% 
% tableWidth = 700;
% tableHeight = 425;
% padding = 40;
% 
% fig = uifigure('Position', [100 100 tableWidth+padding tableHeight+padding]);
% uitable(fig,'Data',T{:,:},'ColumnName',colNames,'Position', [20, 20, tableWidth, tableHeight]);
% if save_fig
%     save_name = strcat(fig_name,'_',EOMtype,'kvd=',num2str(k_),'_MSDsloshMasses_optSpring_table','.png');
%     exportapp(fig, save_name)
% end

% Assume absIndexEps and absIndexSigma are vectors of length 84 (7 rows * 12 columns)
rowNames = {'r1','r2','r3','z1','z2','z3','tan'};
colNames = {'Se2','Se3','Sc2','Sc3','Sg2','Sg3','Te2','Te3','Tc2','Tc3','Tg2','Tg3'};

% Reshape data into 7x12 matrices
% dataEps = reshape(eps, [12, 7])';
% dataSigma = reshape(sigma, [12, 7])';

dataEps = eps;
dataSigma = sigma;
% Create tables
T_eps = array2table(dataEps, 'VariableNames', colNames, 'RowNames', rowNames);
T_sigma = array2table(dataSigma, 'VariableNames', colNames, 'RowNames', rowNames);

% Display in UI tables
tableWidth = 1000;
tableHeight = 200;
padding = 40;

fig1 = uifigure('Position', [100 100 tableWidth+padding tableHeight+padding]);
uitable(fig1, 'Data', T_eps{:,:}, 'ColumnName', colNames, 'RowName', rowNames, ...
        'Position', [20, 20, tableWidth, tableHeight]);

fig2 = uifigure('Position', [200 150 tableWidth+padding tableHeight+padding]);
uitable(fig2, 'Data', T_sigma{:,:}, 'ColumnName', colNames, 'RowName', rowNames, ...
        'Position', [20, 20, tableWidth, tableHeight]);

% Optional saving
if save_fig
    name_eps = fullfile(pwd,strcat(fig_name,  EOMtype, 'P_eps_table.png'));
    name_sigma = fullfile(pwd,strcat(fig_name, EOMtype, 'P_sigma_table.png'));
    exportapp(fig1, name_eps)
    exportapp(fig2, name_sigma)
end



% %% Study on Height
% numSloshMasses = 1
% 
% % Define the enumeration of three types
% % configTypes = {'R', 'Z', 'TAN'};
% configTypes = {'R', 'Z'};
% 
% 
% % Preallocate arrays for results (3 rows for 3 configs)
% % f_obj_eps = zeros(3, N+1);
% % f_obj_sigma = zeros(3, N+1);
% % absIndexEps = zeros(3, 1);
% % absIndexSigma = zeros(3, 1);
% f_obj_eps = zeros(length(configTypes), N);
% f_obj_sigma = zeros(length(configTypes), N+1);
% absIndexEps = zeros(length(configTypes), 1);
% absIndexSigma = zeros(length(configTypes), 1);
% 
% % indexR_all = zeros(numSloshMasses, 2);
% % indexZ_all = zeros(numSloshMasses, 2);
% 
% tic
% idx = 1;
% % for k = 1:length(configTypes)
%     for i=1:numSloshMasses
%         % Pass string as configuration argument
%         % index = PHeightIndexSloshMasses(i, h, R, EOMtype, N, erased_path, Te_array, str, 1, 0, 'vect_value', configTypes{k});
%         % indexSigma = PHeightIndexSloshMasses(i, h, R, EOMtype, N, erased_path, Te_array, str, 0, 1,'abs_value', configTypes{k});
%         [index, indexR, indexZ] = PHeightIndexSloshMasses(i, h, R, EOMtype, N, erased_path, Te_array, str, 1, 0, 'vect_value');
% 
%         % f_obj_eps(k,:)   = index(1,:);
%         % f_obj_sigma(k,:) = [index(2,1:N), indexSigma];
%         % f_obj_eps(i*k + 3*(k-1),:)   = index(1,:);
%         % f_obj_sigma(i*k + 3*(k-1),:) = [index(2,1:N), indexSigma];
% 
%         % f_obj_eps(idx,:)   = index(1,:);
%         % f_obj_sigma(idx,:) = [index(2,1:N), indexSigma];
%         % 
%         % absIndexEps(k)   = sum(abs(f_obj_eps(k,1:N))) / N;
%         % absIndexSigma(k) = sum(abs(f_obj_sigma(k,1:N))) / N;
%         % 
%         % idx = idx + 1;
% 
%         indexR_all(i,:) = indexR(1,:);
%         indexZ_all(i,:) = indexZ(1,:);
% 
%         idsR_all(i,:) = indexR(2,:);
%         idsZ_all(i,:) = indexZ(2,:);
% 
% 
%     end
% % end
% toc
% 
% f_obj_eps = [indexR_all; indexZ_all];
% f_obj_sigma = [idsR_all; idsZ_all];
% 
% 
% % tic
% index = PHeightIndexSloshMasses(1, h, R, EOMtype, N, erased_path, Te_array, str, 1, 0, 'vect_value', 'tan');
% indexSigma = PHeightIndexSloshMasses(1, h, R, EOMtype, N, erased_path, Te_array, str, 0, 1,'abs_value', 'tan');
% 
% f_obj_eps = [f_obj_eps, index(1,:)];
% f_obj_sigma = [f_obj_sigma, [index(2,1:N), indexSigma]];
% 
% toc
%% Graphics
% label_size  = 14;
% axis_size   = 14;
% legend_size = 14;
% line_width  = 2.5;
% num_cols    = 3;
% 
% legend_text = {};
% figure
% hold on
% grid on
% box on
% xticks(alphan_range(1):0.1:alphan_range(end))
% for j = 1:length(w_range)
%     plot(alphan_range,f_obj_eps(:,end,j),'LineWidth',2)
%     legend_text{j} = ['w=',num2str(w_range(j))];
%     % plot(alphan_range,absIndexEps(:,j)','--k')
% end
% xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$f_{obj,\epsilon}$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(legend_text,'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',1);
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% if save_fig
%     save_name = strcat(fig_name,'Fobj_eps_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% legend_text = {};
% figure
% hold on
% grid on
% box on
% xticks(alphan_range(1):0.1:alphan_range(end))
% for j = 1:length(w_range)
%     plot(alphan_range,f_obj_sigma(:,end,j),'LineWidth',2)
%     legend_text{j} = ['w=',num2str(w_range(j))];
%     % plot(alphan_range,absIndexSigma(:,j)','--k')
% end
% xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$f_{obj,\sigma}$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(legend_text,'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',1);
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% if save_fig
%     save_name = strcat(fig_name,'Fobj_sigma_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% legend_text = {};
% figure
% hold on
% grid on
% box on
% xticks(alphan_range(1):0.1:alphan_range(end))
% for j = 1:length(w_range)
%     plot(alphan_range,f_obj_eps(:,end,j)+f_obj_sigma(:,end,j),'LineWidth',2)
%     legend_text{j} = ['w=',num2str(w_range(j))];
%     % plot(alphan_range,absIndexSigma(:,j)','--k')
% end
% xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
% ylabel('$f_{obj,\epsilon}+f_{obj,\sigma}$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
% leg = legend(legend_text,'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',1);
% set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
% % if save_fig
% %     save_name = strcat(fig_name,'Fobj_eps_sigma','.png');
% %     set(gcf,'PaperPositionMode','auto')
% %     print(save_name,'-dpng','-r0')
% % end
% 
% 
% 
% fig = figure()
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
% 
% for j = 1:length(w_range)
%     subplot(2,3,j)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     ylim([0 80])
%     plot(alphan_range,abs(f_obj_eps(:,1:6,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$|\epsilon|$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{1:6},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% 
%     subplot(2,3,j+3)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     ylim([0 80])
%     plot(alphan_range,abs(f_obj_eps(:,7:12,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$|\epsilon|$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{7:12},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% end
% if save_fig
%     save_name = strcat(fig_name,'Allmotions_abs_eps_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% fig = figure()
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20]);
% 
% for j = 1:length(w_range)
%     subplot(2,3,j)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     % ylim([0 150])
%     ylim([0 50])
%     plot(alphan_range,abs(f_obj_sigma(:,1:6,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$|\sigma|$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{1:6},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% 
%     subplot(2,3,j+3)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     % ylim([0 150])
%     ylim([0 50])
%     plot(alphan_range,abs(f_obj_sigma(:,7:12,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$|\sigma|$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{7:12},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% end
% if save_fig
%     save_name = strcat(fig_name,'Allmotions_abs_sigma_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% fig = figure()
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
% 
% for j = 1:length(w_range)
%     subplot(2,3,j)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     ylim([-80 40])
%     plot(alphan_range,(f_obj_eps(:,1:6,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$\epsilon$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{1:6},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% 
%     subplot(2,3,j+3)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     ylim([-80 40])
%     plot(alphan_range,(f_obj_eps(:,7:12,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$\epsilon$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{7:12},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% end
% if save_fig
%     save_name = strcat(fig_name,'Allmotions_eps_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% fig = figure()
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% pos = get(fig,'Position');
% set(fig,'Units','Normalized');
% set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20]);
% 
% for j = 1:length(w_range)
%     subplot(2,3,j)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     % ylim([-150 150])
%     ylim([-50 50])
%     plot(alphan_range,(f_obj_sigma(:,1:6,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$\sigma$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{1:6},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% 
%     subplot(2,3,j+3)
%     hold on
%     grid on
%     box on
%     xticks(alphan_range(1):0.1:alphan_range(end))
%     % ylim([-150 150])
%     ylim([-50 50])
%     plot(alphan_range,(f_obj_sigma(:,7:12,j)),'LineWidth',2)
%     xlabel('$\alpha_n$','FontSize', label_size, 'Interpreter', 'latex');
%     ylabel('$\sigma$ [\%]','FontSize', label_size, 'Interpreter', 'latex');
%     leg = legend(motion_array_new{7:12},'Fontsize',label_size,'interpreter', 'latex','Location','best','NumColumns',num_cols);
%     leg.ItemTokenSize = [8,8];
%     set(gca, 'FontSize', axis_size, 'TickLabelInterpreter', 'latex');
%     title(['w=',num2str(w_range(j))],'interpreter', 'latex');
% end
% if save_fig
%     save_name = strcat(fig_name,'Allmotions_sigma_',num2str(numSloshMasses),'_slosh_masses','.png');
%     set(gcf,'PaperPositionMode','auto')
%     print(save_name,'-dpng','-r0')
% end
% 
% 
% 
% %% Save csv files
% 
% if save_csv
% 
%     for j = 1:length(w_range)
% 
%         A = [motion_array    , "f_obj_eps", motion_array    , "f_obj_sigma";
%              motion_array_new, "f_obj_eps", motion_array_new, "f_obj_sigma";
%                      f_obj_eps(:,:,j)     ,         f_obj_sigma(:,:,j)     ];
%         save_name = strcat(fig_name,'Allmotions_eps_sigma_alphan_',num2str(alphan_range(1)),'_',num2str(alphan_range(end)),'_w_',num2str(w_range(j)),'_',num2str(length(alphan_range)),'_',num2str(numSloshMasses),'_slosh_masses','.csv');
%         writematrix(A,save_name,'Delimiter','comma');
% 
%     end
% 
% end
% 



