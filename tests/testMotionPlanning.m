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

terrain = motionplanning.environment.generateVoxelTerrain(cfg.environment);

verifySize(testCase, terrain.X, [20, 20]);
verifySize(testCase, terrain.Y, [20, 20]);
verifySize(testCase, terrain.Z, [20, 20]);
verifyGreaterThanOrEqual(testCase, min(terrain.Z(:)), 0);
verifyEqual(testCase, terrain.maxX, 30);
verifyEqual(testCase, terrain.maxY, 30);
end

function testStateValidityUsesBodyCenterClearance(testCase)
cfg = motionplanning.config.defaultConfig();
robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);

[isValid, bodyZ] = motionplanning.planning.isStateValid([12, 12], terrain, robot, 0);

verifyTrue(testCase, isValid);
verifyEqual(testCase, bodyZ, robot.clearance + robot.bodyHeight / 2, 'AbsTol', 1e-12);
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
cfg.planning.maxNodes = 100;
cfg.planning.stepSize = 4;
cfg.planning.goalBias = 1;
cfg.planning.goalTolerance = cfg.planning.stepSize;
cfg.planning.edgeCheckResolution = 1;

robot = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = flatTerrain(40, 41, 0);
startState = [12, 12];
goalState = [28, 28];

[pathXY, pathZ, info] = motionplanning.planning.planRRT( ...
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
    'gridSpacing', domainSize / max(resolution - 1, 1));
end
