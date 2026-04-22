function tests = testMotionPlanning
%TESTMOTIONPLANNING Focused tests for simulator modules.

tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'src'));
end

function testGeneratedTerrainShape(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.environment.gridResolution = 20;
cfg.environment.domainSize = 30;
cfg.environment.topologyFeatureCount = 6;
cfg.environment.maxTerrainHeight = 20;

terrain = motionplanning.environment.generateVoxelTerrain(cfg.environment);

verifySize(testCase, terrain.X, [20, 20]);
verifySize(testCase, terrain.Y, [20, 20]);
verifySize(testCase, terrain.Z, [20, 20]);
verifyGreaterThanOrEqual(testCase, min(terrain.Z(:)), 0);
verifyEqual(testCase, terrain.maxX, 30);
verifyEqual(testCase, terrain.maxY, 30);
verifySize(testCase, terrain.xVec, [1, 20]);
verifySize(testCase, terrain.yVec, [1, 20]);
verifyLessThanOrEqual(testCase, max(terrain.Z(:)), cfg.environment.maxTerrainHeight + cfg.environment.blockHeight);
end

function testDefaultEnvironmentIsLargeAndUnguided(testCase)
cfg = motionplanning.config.defaultConfig();

verifyGreaterThanOrEqual(testCase, cfg.environment.domainSize, 180);
verifyGreaterThanOrEqual(testCase, cfg.environment.gridResolution, 200);
verifyGreaterThan(testCase, cfg.environment.topologyFeatureCount, 0);
verifyEqual(testCase, cfg.environment.diagonalValleyDepth, 0);
verifyGreaterThanOrEqual(testCase, cfg.planning.maxNodes, 2500);
verifyEqual(testCase, cfg.planning.algorithm, 'weighted_astar');
end

function testLowFrequencyTerrainDoesNotCreateLargeKernel(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.environment.gridResolution = 30;
cfg.environment.domainSize = 100;
cfg.environment.frequency = 0.001;
cfg.environment.topologyFeatureCount = 8;

terrain = motionplanning.environment.generateVoxelTerrain(cfg.environment);

verifySize(testCase, terrain.Z, [30, 30]);
verifyFalse(testCase, any(isnan(terrain.Z(:))));
end

function testStateValidityUsesBodyCenterClearance(testCase)
cfg = motionplanning.config.defaultConfig();
robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);

[isValid, bodyZ] = motionplanning.planning.isStateValid([12, 12], terrain, robot, 0);

verifyTrue(testCase, isValid);
verifyEqual(testCase, bodyZ, robot.clearance + robot.bodyHeight / 2, 'AbsTol', 1e-12);
end

function testTripodStaticStability(testCase)
cfg = motionplanning.config.defaultConfig();
robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);

[isValid, ~, info] = motionplanning.planning.isStateValid([20, 20], terrain, robot, 0);

verifyTrue(testCase, isValid);
verifyTrue(testCase, all(info.stabilityInfo.phaseStable));
verifyGreaterThan(testCase, min(info.stabilityInfo.phaseMargins), robot.stabilityMargin);
end

function testBoundaryFeetAreRejected(testCase)
cfg = motionplanning.config.defaultConfig();
robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);

[isValid, ~, info] = motionplanning.planning.isStateValid([5, 5], terrain, robot, 0);

verifyFalse(testCase, isValid);
verifyEqual(testCase, info.reason, 'leg_unreachable');
end

function testRRTReturnsValidFlatTerrainPath(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.algorithm = 'rrt';
cfg.planning.maxNodes = 100;
cfg.planning.stepSize = 4;
cfg.planning.goalBias = 1;
cfg.planning.goalTolerance = cfg.planning.stepSize;
cfg.planning.edgeCheckResolution = 1;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

[pathXY, pathZ, info] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info.status, 'goal_reached');
verifyFalse(testCase, isempty(pathXY));
verifyEqual(testCase, pathXY(1, :), startState);
verifyEqual(testCase, pathXY(end, :), goalState);
verifyEqual(testCase, pathZ, ones(size(pathZ)) * (robot.clearance + robot.bodyHeight / 2), 'AbsTol', 1e-12);

for idx = 1:(size(pathXY, 1) - 1)
    edgeValid = motionplanning.planning.isEdgeValid( ...
        pathXY(idx, :), pathXY(idx + 1, :), terrain, robot, cfg.planning);
    verifyTrue(testCase, edgeValid);
end
end

function testInformedRRTStarReturnsValidFlatTerrainPath(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.algorithm = 'informed_rrt_star';
cfg.planning.maxNodes = 120;
cfg.planning.stepSize = 4;
cfg.planning.goalBias = 1;
cfg.planning.goalTolerance = cfg.planning.stepSize;
cfg.planning.edgeCheckResolution = 1;
cfg.planning.rewireRadius = 16;
cfg.planning.solutionCheckInterval = 1;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

[pathXY, pathZ, info] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info.status, 'goal_reached');
verifyFalse(testCase, isempty(pathXY));
verifyEqual(testCase, pathXY(1, :), startState);
verifyEqual(testCase, pathXY(end, :), goalState);
verifyEqual(testCase, pathZ, ones(size(pathZ)) * (robot.clearance + robot.bodyHeight / 2), 'AbsTol', 1e-12);
end

function testBITStarReturnsValidFlatTerrainPath(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.algorithm = 'bit_star';
cfg.planning.edgeCheckResolution = 1;
cfg.planning.bitBatchSize = 10;
cfg.planning.bitMaxBatches = 2;
cfg.planning.bitConnectionRadius = 40;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

[pathXY, pathZ, info] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info.status, 'goal_reached');
verifyFalse(testCase, isempty(pathXY));
verifyEqual(testCase, pathXY(1, :), startState);
verifyEqual(testCase, pathXY(end, :), goalState);
verifyEqual(testCase, pathZ, ones(size(pathZ)) * (robot.clearance + robot.bodyHeight / 2), 'AbsTol', 1e-12);
end

function testSpatialNearestMatchesBruteForce(testCase)
terrain = flatTerrain(40, 41, 0);
nodes = [
    5 5
    10 10
    12 25
    30 8
    35 35
];
query = [13, 23];

spatialIndex = motionplanning.planning.createSpatialIndex(terrain, 8);
for idx = 1:size(nodes, 1)
    spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, idx, nodes(idx, :));
end

spatialNearest = motionplanning.planning.nearestNodeSpatial( ...
    spatialIndex, nodes, size(nodes, 1), query);
bruteNearest = motionplanning.planning.nearestNode(nodes, size(nodes, 1), query);

verifyEqual(testCase, spatialNearest, bruteNearest);
end

function terrain = flatTerrain(domainSize, resolution, height)
xVec = linspace(0, domainSize, resolution);
yVec = linspace(0, domainSize, resolution);
[X, Y] = meshgrid(xVec, yVec);
terrain = struct( ...
    'X', X, ...
    'Y', Y, ...
    'Z', ones(resolution, resolution) * height, ...
    'minX', 0, ...
    'maxX', domainSize, ...
    'minY', 0, ...
    'maxY', domainSize, ...
    'resolution', resolution, ...
    'xVec', xVec, ...
    'yVec', yVec, ...
    'gridSpacing', domainSize / max(resolution - 1, 1));
end

%% ---- Weighted A* / Hybrid planner tests ----

function testWeightedAStarReturnsValidFlatTerrainPath(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.algorithm = 'weighted_astar';
cfg.planning.edgeCheckResolution = 1;
cfg.planning.astarEpsilon = 3.0;
cfg.planning.rrtConnectMaxIter = 100;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

rng(99);  % fix seed for reproducible smoothing
[pathXY, pathZ, info] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info.status, 'goal_reached');
verifyFalse(testCase, isempty(pathXY));
verifyEqual(testCase, pathXY(1, :), startState, 'AbsTol', 1e-6);
verifyEqual(testCase, pathXY(end, :), goalState, 'AbsTol', 1e-6);
% Path should have more waypoints after densification
verifyGreaterThanOrEqual(testCase, size(pathXY, 1), 3);
% Z should be at clearance height on flat terrain
expectedZ = robot.clearance + robot.bodyHeight / 2;
verifyEqual(testCase, pathZ, ones(size(pathZ)) * expectedZ, 'AbsTol', 0.5);
end

function testWeightedAStarIsDeterministic(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.algorithm = 'weighted_astar';
cfg.planning.edgeCheckResolution = 1;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

% Fix seed before each call so random shortcutting is deterministic
rng(42);
[pathXY1, ~, info1] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);
rng(42);
[pathXY2, ~, info2] = motionplanning.planning.solvePath( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info1.status, 'goal_reached');
verifyEqual(testCase, info2.status, 'goal_reached');
verifyEqual(testCase, pathXY1, pathXY2, 'AbsTol', 1e-12);
end

function testTraversabilityMapOnFlatTerrain(testCase)
cfg = motionplanning.config.defaultConfig();
robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);

[traversable, costMap] = motionplanning.planning.buildTraversabilityMap( ...
    terrain, robot, cfg.planning);

verifySize(testCase, traversable, [41, 41]);
verifySize(testCase, costMap, [41, 41]);
% Interior cells far from boundary should be traversable on flat terrain
verifyTrue(testCase, traversable(21, 21));
% Cost should be finite for traversable cells
verifyTrue(testCase, isfinite(costMap(21, 21)));
% Non-traversable cells should have Inf cost
blockedCells = costMap(~traversable);
if ~isempty(blockedCells)
    verifyTrue(testCase, all(isinf(blockedCells)));
end
end

function testRRTConnectOnFlatTerrain(testCase)
cfg = motionplanning.config.defaultConfig();
cfg.planning.rrtConnectStepSize = 3.0;
cfg.planning.rrtConnectMaxIter = 200;
cfg.planning.edgeCheckResolution = 1;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [15, 15];
goalState = [25, 25];

[pathXY, pathZ, info] = motionplanning.planning.planRRTConnect( ...
    startState, goalState, terrain, robot, cfg.planning);

verifyEqual(testCase, info.status, 'goal_reached');
verifyFalse(testCase, isempty(pathXY));
verifyEqual(testCase, pathXY(1, :), startState, 'AbsTol', 1.0);
% Path Z should be at clearance height on flat terrain
expectedZ = robot.clearance + robot.bodyHeight / 2;
verifyEqual(testCase, pathZ, ones(size(pathZ)) * expectedZ, 'AbsTol', 2.0);
end

