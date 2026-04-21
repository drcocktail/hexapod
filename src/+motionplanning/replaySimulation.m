function replaySimulation(filename)
%REPLAYSIMULATION Load a saved simulation result and immediately run the UI playback
%   Does not re-compute paths. Just starts the renderer.

if nargin < 1
    filename = 'simulation_result.mat';
end

if ~isfile(filename)
    error('motionplanning:replay:FileNotFound', 'Could not find %s', filename);
end

data = load(filename);
if ~isfield(data, 'saveStruct')
    error('motionplanning:replay:InvalidData', 'The file does not contain a valid saveStruct.');
end
result = data.saveStruct;

fprintf('Simulation data loaded. Launching interactive playback...\n');

cfg = motionplanning.config.defaultConfig();
% Force enable animations automatically for replay purposes
cfg.visualization.enabled = true;
cfg.visualization.animate = true;

[fig, ax] = motionplanning.visualization.renderScene( ...
    result.terrain, result.pathXY, result.pathZ, result.start, result.goal, result.seed, cfg.visualization);

motionplanning.visualization.animateHexapod(ax, result.pathXY, result.pathZ, result.terrain, result.robot, cfg.visualization);

end
