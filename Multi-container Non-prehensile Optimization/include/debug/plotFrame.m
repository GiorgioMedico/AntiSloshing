function plotFrame(origin, R, label)
    quiver3(origin(1), origin(2), origin(3), R(1,1), R(2,1), R(3,1), 0.1, 'r', 'LineWidth', 2); % X-axis
    quiver3(origin(1), origin(2), origin(3), R(1,2), R(2,2), R(3,2), 0.1, 'g', 'LineWidth', 2); % Y-axis
    quiver3(origin(1), origin(2), origin(3), R(1,3), R(2,3), R(3,3), 0.1, 'b', 'LineWidth', 2); % Z-axis
    text(origin(1), origin(2), origin(3), ['  ', label], 'FontSize', 12, 'FontWeight', 'bold');
end
