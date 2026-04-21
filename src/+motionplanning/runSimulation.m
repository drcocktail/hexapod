function result = runSimulation(varargin)
%RUNSIMULATION Configure, plan, and optionally animate a hexapod run.

cfg = motionplanning.config.defaultConfig();
cfg = parseOverrides(cfg, varargin{:});

fprintf('Initializing Hexapod Volumetric Simulator...\n');

if isempty(cfg.seed)
    simSeed = randi(cfg.seedRange);
else
    simSeed = cfg.seed;
end
rng(simSeed);
fprintf('Procedural Generation Seed: %d\n', simSeed);

robot = motionplanning.robot.createHexapodRobot(cfg.robot);

fprintf('Generating Voxelized fBm Terrain...\n');
terrain = motionplanning.environment.generateVoxelTerrain(cfg.environment);

[startNode, goalNode] = resolveEndpoints(cfg, robot, terrain);
spawnFlatRadius = max(cfg.environment.spawnFlatRadius, ...
    robot.nominalFootRadius + terrain.gridSpacing);
terrain.Z = motionplanning.environment.flattenRegion( ...
    terrain.Z, terrain.X, terrain.Y, startNode, spawnFlatRadius);
terrain.Z = motionplanning.environment.flattenRegion( ...
    terrain.Z, terrain.X, terrain.Y, goalNode, spawnFlatRadius);

fprintf('Executing %s planning with volumetric validation...\n', char(cfg.planning.algorithm));
[pathXY, pathZ, plannerInfo] = motionplanning.planning.solvePath( ...
    startNode, goalNode, terrain, robot, cfg.planning);

if isempty(pathXY)
    error('motionplanning:planner:NoPath', ...
        'Solver failed to find a valid path. Status: %s. Re-run with a different seed or adjust planner/environment options.', ...
        plannerInfo.status);
end

fig = [];
ax = [];
if cfg.visualization.enabled
    fprintf('Rendering 3D Environment...\n');
    [fig, ax] = motionplanning.visualization.renderScene( ...
        terrain, pathXY, pathZ, startNode, goalNode, simSeed, cfg.visualization);

    if cfg.visualization.animate
        fprintf('Executing 3D Kinematic Animation...\n');
        motionplanning.visualization.animateHexapod(ax, pathXY, pathZ, terrain, robot, cfg.visualization);
    end
end

fprintf('Simulation Execution Completed.\n');

result = struct( ...
    'seed', simSeed, ...
    'robot', robot, ...
    'terrain', terrain, ...
    'start', startNode, ...
    'goal', goalNode, ...
    'pathXY', pathXY, ...
    'pathZ', pathZ, ...
    'plannerInfo', plannerInfo, ...
    'figure', fig, ...
    'axes', ax);
end

function [startNode, goalNode] = resolveEndpoints(cfg, robot, terrain)
margin = robot.nominalFootRadius + cfg.environment.endpointMargin;
if isempty(cfg.start)
    startNode = [margin, margin];
else
    startNode = cfg.start;
end

if isempty(cfg.goal)
    goalNode = [terrain.maxX - margin, terrain.maxY - margin];
else
    goalNode = cfg.goal;
end
end

function cfg = parseOverrides(cfg, varargin)
if isempty(varargin)
    return;
end

if numel(varargin) == 1 && isstruct(varargin{1})
    cfg = mergeStruct(cfg, varargin{1});
    return;
end

if mod(numel(varargin), 2) ~= 0
    error('motionplanning:config:InvalidOverrides', ...
        'Overrides must be provided as name-value pairs or a struct.');
end

for idx = 1:2:numel(varargin)
    name = lower(char(varargin{idx}));
    value = varargin{idx + 1};
    switch name
        case 'seed'
            cfg.seed = value;
        case 'render'
            cfg.visualization.enabled = logical(value);
        case 'animate'
            cfg.visualization.animate = logical(value);
        case 'start'
            cfg.start = value;
        case 'goal'
            cfg.goal = value;
        case 'maxnodes'
            cfg.planning.maxNodes = value;
        case 'stepsize'
            cfg.planning.stepSize = value;
        case 'goalbias'
            cfg.planning.goalBias = value;
        case 'algorithm'
            cfg.planning.algorithm = value;
        case 'planner'
            cfg.planning.algorithm = value;
        case 'bitbatchsize'
            cfg.planning.bitBatchSize = value;
        case 'bitmaxbatches'
            cfg.planning.bitMaxBatches = value;
        otherwise
            error('motionplanning:config:UnknownOverride', ...
                'Unknown runSimulation override: %s', varargin{idx});
    end
end
end

function base = mergeStruct(base, overrides)
fields = fieldnames(overrides);
for idx = 1:numel(fields)
    key = fields{idx};
    if isstruct(overrides.(key)) && isfield(base, key) && isstruct(base.(key))
        base.(key) = mergeStruct(base.(key), overrides.(key));
    else
        base.(key) = overrides.(key);
    end
end
end
