function [fig, ax, handles] = renderScene(terrain, pathXY, pathZ, startState, goalState, seed, options)
%RENDERSCENE Draw terrain, start/goal markers, and the planned path.

fig = figure( ...
    'Name', sprintf('Hexapod Simulator (Seed: %d)', seed), ...
    'Color', [0.1, 0.1, 0.15], ...
    'Position', options.figurePosition);
ax = axes('Parent', fig, 'Color', [0.1, 0.1, 0.15]);
hold(ax, 'on');
grid(ax, 'on');
view(ax, [-45, 45]);

light(ax, 'Position', [0, 0, 100], 'Style', 'local', 'Color', [0.9, 0.9, 1.0]);
light(ax, 'Position', [terrain.maxX, terrain.maxY, 50], 'Style', 'local', 'Color', [1.0, 0.8, 0.6]);
lighting(ax, 'flat');

handles.terrain = surf(ax, terrain.X, terrain.Y, terrain.Z, ...
    'EdgeColor', [0.2, 0.2, 0.2], ...
    'EdgeAlpha', 0.3, ...
    'FaceColor', [0.3, 0.4, 0.3]);

handles.path = plot3(ax, pathXY(:, 1), pathXY(:, 2), pathZ, ...
    'w--', 'LineWidth', 2);
handles.start = plot3(ax, startState(1), startState(2), pathZ(1), ...
    'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
handles.goal = plot3(ax, goalState(1), goalState(2), pathZ(end), ...
    'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

axis(ax, 'equal');
zMax = max([terrain.Z(:); pathZ(:)]) + 8;
axis(ax, [terrain.minX, terrain.maxX, terrain.minY, terrain.maxY, 0, max(25, zMax)]);
xlabel(ax, 'X Axis', 'Color', 'w');
ylabel(ax, 'Y Axis', 'Color', 'w');
zlabel(ax, 'Elevation', 'Color', 'w');
set(ax, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
end
