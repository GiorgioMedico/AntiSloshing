f_s = figure();
set(f_s, 'Name', 'S Results', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);

% Use tiled layout for better control
t = tiledlayout(2,2, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

% Overarching title
title(t, 'State Variables and Control Input Optimization Results', 'Interpreter','latex', 'FontSize', 16)
% s
nexttile
hold on
grid on
box on
plot(tvec,s_sol,LineWidth=2);
legend('s',Interpreter="latex", Location='northeast')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
ylabel("s",Interpreter="latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
xlim([0 t_end])
title('s', 'Interpreter', 'latex')
line([0 t_end],[s_min s_min],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')
line([0 t_end],[s_max s_max],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')

% s_dot
nexttile
hold on
grid on
box on
plot(tvec,s_dot_sol,LineWidth=2);
legend('$\dot s$',Interpreter="latex", Location='northeast')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
ylabel('$\dot s$',Interpreter="latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
xlim([0 t_end])
title('$\dot s$', 'Interpreter', 'latex')
% line([0 t_end],[s_dot_min s_dot_min],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')
% line([0 t_end],[s_dot_max s_dot_max],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')

% s_ddot
nexttile
hold on
grid on
box on
plot(tvec,s_ddot_sol,LineWidth=2);
legend('$\ddot s$',Interpreter="latex", Location='northeast')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
ylabel('$\ddot s$',Interpreter="latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
xlim([0 t_end])
title('$\ddot s$', 'Interpreter', 'latex')
% line([0 t_end],[s_ddot_min s_ddot_min],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')
% line([0 t_end],[s_ddot_max s_ddot_max],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')

% Jerk
nexttile
hold on
grid on
box on
plot(tvec,U_sol,LineWidth=2)
legend('$\stackrel{...}{s}$',Interpreter="latex", Location='northeast')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
% plot(tvec,ones(1,length(tvec)),LineStyle="--");
% plot(tvec,-1*ones(1,length(tvec)),LineStyle="--",Color='black');
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$\stackrel{...}{s}$',Interpreter="latex")
xlim([0 t_end])
title('$\stackrel{...}{s}$', 'Interpreter', 'latex')
line([0 t_end],[-s_j_lim -s_j_lim],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')
line([0 t_end],[s_j_lim s_j_lim],'Color','red','LineStyle','--','LineWidth',2,'HandleVisibility','off')