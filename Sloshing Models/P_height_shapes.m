%% Plot PEN sloshing height formulations surfaces comparison

close all
clear all
clc

addpath(genpath("odes"));


% Parametri
R = 0.049;               % Raggio
h = 0.078;
[~, ~, m_tot, ~, xi_1n, zitan, mn, ~, ~, ~, ln, ~, ~, ~, wn] = nModeParameters(R, h, 1);

K = 2*R / (xi_1n^2 - 1);

beta_max = deg2rad(45);                      % max allowed beta
cos_lim = cos(beta_max);                     % limit on |cos(phi_x)cos(phi_y)|
cos_phi_max = sqrt(cos_lim);                 % assume cos(phi_x) = cos(phi_y)
phi_max = acos(cos_phi_max);                 % resulting phi_max (radians)

% phi_max = pi/6;
% Range angoli
phi_x = linspace(-phi_max, phi_max, 30);
phi_y = linspace(-phi_max, phi_max, 30);
[PHI_X, PHI_Y] = meshgrid(phi_x, phi_y);

% -------- Eq.1 --------
arg = abs(-cos(PHI_X) .* cos(PHI_Y));
arg = min(max(arg, 0), 1);
beta = acos(arg);
eta1 = R * tan(beta)/R;



% -------- Eq.2 --------
eta2 = K * sqrt(2 * (1 - cos(PHI_X) .* cos(PHI_Y)))/R;

% -------- Eq.3 --------
eta3 = K * sqrt(sin(PHI_Y).^2 .* cos(PHI_X).^2 + sin(PHI_X).^2)/R;

% -------- Grafico --------
fig = figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(fig,'Position');
set(fig,'Units','Normalized');
set(fig,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
hold on;
box on

% Superfici
s3 = surf(rad2deg(PHI_X), rad2deg(PHI_Y), eta3, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
s2 = surf(rad2deg(PHI_X), rad2deg(PHI_Y), eta2, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
s1 = surf(rad2deg(PHI_X), rad2deg(PHI_Y), eta1, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

% Colori personalizzati
s1.FaceColor = 'r';
s2.FaceColor = 'g';
s3.FaceColor = 'b';

% title('$\bar{\eta}$ Formulation Comparison','Interpreter','latex','FontSize', 18);
xlabel('$\phi_{x1} [^\circ]$', 'Interpreter', 'latex', 'FontSize', 46);
ylabel('$\phi_{y1} [^\circ]$', 'Interpreter', 'latex', 'FontSize', 46);
zlabel('$\bar{\eta}/R$', 'Interpreter', 'latex', 'FontSize', 46);
legend({'PEN-rad', ...
        'PEN-vert', ...
        'PEN-tan'}, ...
        'Interpreter','latex', 'FontSize', 34);
set(gca, 'FontSize', 44, 'TickLabelInterpreter', 'latex');
% zlim([0,1])

grid on;
% grid minor
view(45,30);
hold off;
save_name = strcat("PEN_height_surf",'.pdf');
set(fig, 'Renderer', 'painters')  % Forces vector output where possible
print('-dpdf', '-fillpage', save_name)
% exportgraphics(fig, save_name, 'ContentType', 'vector')


%%
cos_lim = cos(beta_max);                     % limit on |cos(phi_x)cos(phi_y)|
cos_phi_max = sqrt(cos_lim);                 % assume cos(phi_x) = cos(phi_y)
phi_max = acos(cos_phi_max);                 % resulting phi_max (radians)

% phi_max = pi/6;
% Range angoli
phi_y = linspace(-phi_max, phi_max, 100);

% -------- Eq.1 --------
arg = abs(-cos(phi_y));
arg = min(max(arg, 0), 1);
beta = acos(arg);
eta1_x0 = R * tan(beta)/R;

% -------- Eq.2 --------
eta2_x0 = K * sqrt(2 * (1 - cos(phi_y)))/R;

% -------- Eq.3 --------
eta3_x0 = K * sqrt(sin(phi_y).^2)/R;

% --- Section along phi_x = 0 (varying phi_y)
% idx_x0 = find(abs(phi_x) == min(abs(phi_x)));  % index where phi_x ~ 0
% idx_x0 = find(phi_x==0);  % index where phi_x ~ 0
% eta1_x0 = eta1(:, idx_x0);
% eta2_x0 = eta2(:, idx_x0);
% eta3_x0 = eta3(:, idx_x0);
phi_y_deg = rad2deg(phi_y);

% --- Plot the sections
f2 = figure;
pos = get(f2,'Position');
set(f2,'Units','Normalized');
set(f2,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])
hold on;
box on
set(f2, 'Units', 'Normalized', 'OuterPosition', [0.1 0.1 0.8 0.6]);

% --- Plot 1: phi_x = 0 (vs phi_y)
plot(phi_y_deg, eta3_x0, 'b', 'LineWidth', 4.5);
hold on;
plot(phi_y_deg, eta2_x0, 'g', 'LineWidth', 4.5);
plot(phi_y_deg, eta1_x0, 'r', 'LineWidth', 4.5); 
legend({'PEN-rad', ...
        'PEN-vert', ...
        'PEN-tan'}, ...
        'Interpreter','latex', 'FontSize', 34);
xlabel('$\phi_{y1} [^\circ]$', 'Interpreter', 'latex', 'FontSize', 46);
ylabel('$\bar{\eta}/R$', 'Interpreter', 'latex', 'FontSize', 46);
xlim([-rad2deg(phi_max),rad2deg(phi_max)])

grid on; box on;
set(gca, 'FontSize', 44, 'TickLabelInterpreter', 'latex');
save_name = strcat("PEN_height_section",'.pdf');
print('-dpdf', '-fillpage', save_name)

%%

% Find indices where phi_x > 0
idx_pos_x = find(phi_x > 0);
PHI_X_pos = PHI_X(:, idx_pos_x);
PHI_Y_pos = PHI_Y(:, idx_pos_x);
eta1_pos = eta1(:, idx_pos_x);
eta2_pos = eta2(:, idx_pos_x);
eta3_pos = eta3(:, idx_pos_x);

% Plot 3D surface just for phi_x > 0
figure;
hold on;
surf(rad2deg(PHI_X_pos), rad2deg(PHI_Y_pos), eta1_pos, ...
     'FaceAlpha', 0.7, 'EdgeColor', 'none', 'FaceColor', 'r');
surf(rad2deg(PHI_X_pos), rad2deg(PHI_Y_pos), eta2_pos, ...
     'FaceAlpha', 0.7, 'EdgeColor', 'none', 'FaceColor', 'g');
surf(rad2deg(PHI_X_pos), rad2deg(PHI_Y_pos), eta3_pos, ...
     'FaceAlpha', 0.7, 'EdgeColor', 'none', 'FaceColor', 'b');

xlabel('$\phi_x$ [°]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('$\phi_y$ [°]', 'Interpreter', 'latex', 'FontSize', 16);
zlabel('$\bar{\eta}/R$', 'Interpreter', 'latex', 'FontSize', 16);
legend({'PEN-tan', 'PEN-vert', 'PEN-rad'}, ...
        'Interpreter','latex', 'FontSize', 14);
grid on;
view(270,0);
box on;
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');

