f_diag = figure();
set(f_diag, 'Name', 'Solver Diagnostics', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 480]);

t = tiledlayout(1,3, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(t, 'Solver Diagnostics', 'Interpreter', 'latex', 'FontSize', 16);

% Objective
nexttile
semilogy(sol.stats.iterations.obj, "LineWidth", 2);
title("Objective", 'Interpreter', 'latex');
xlabel("Iteration", 'Interpreter', 'latex');
ylabel("Objective value", 'Interpreter', 'latex');
xlim([0,length(sol.stats.iterations.obj)])
grid on;

% Jacobian Sparsity
nexttile
spy(sol.value(jacobian(opti.g, opti.x)));
title("Constraint Jacobian Sparsity", 'Interpreter', 'latex');
xlabel("Decision Variables", 'Interpreter', 'latex');
ylabel("Constraints", 'Interpreter', 'latex');
grid on;

% Feasibility
nexttile
semilogy(sol.stats.iterations.inf_du, "LineWidth", 2);
hold on;
semilogy(sol.stats.iterations.inf_pr, "LineWidth", 2);
xlabel("Iteration", 'Interpreter', 'latex');
ylabel("Infeasibility", 'Interpreter', 'latex');
xlim([0,length(sol.stats.iterations.inf_pr)])
legend("Dual", "Primal", 'Location', 'best', 'Interpreter', 'latex');
grid on;
title("Feasibility (log scale)", 'Interpreter', 'latex');