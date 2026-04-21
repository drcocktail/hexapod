function cfg = defaultConfig()
%DEFAULTCONFIG Return simulator defaults in one editable structure.

cfg.seed = [];
cfg.seedRange = 10000;

cfg.robot.bodyRadius = 4.0;
cfg.robot.bodyHeight = 2.0;
cfg.robot.clearance = 3.5;
cfg.robot.L_coxa = 1.5;
cfg.robot.L_femur = 4.5;
cfg.robot.L_tibia = 5.5;
cfg.robot.maxStepZ = 6.0;
cfg.robot.maxPitch = 25 * pi / 180;
cfg.robot.maxRoll = 25 * pi / 180;
cfg.robot.stabilityMargin = 0.25;

cfg.environment.gridResolution = 80;
cfg.environment.domainSize = 60;
cfg.environment.octaves = 4;
cfg.environment.persistence = 0.5;
cfg.environment.frequency = 0.05;
cfg.environment.amplitude = 12.0;
cfg.environment.blockHeight = 2.0;
cfg.environment.valleyInfluence = 6.0;
cfg.environment.valleyDepth = 10.0;
cfg.environment.endpointMargin = 2.0;
cfg.environment.spawnFlatRadius = cfg.robot.bodyRadius * 1.5;

cfg.start = [];
cfg.goal = [];

cfg.planning.maxNodes = 1500;
cfg.planning.stepSize = 3.0;
cfg.planning.goalBias = 0.15;
cfg.planning.goalTolerance = cfg.planning.stepSize;
cfg.planning.edgeCheckResolution = 1.0;
cfg.planning.nearestMode = 'spatial_hash';
cfg.planning.spatialCellSize = cfg.planning.stepSize * 4;

cfg.visualization.enabled = true;
cfg.visualization.animate = true;
cfg.visualization.framesPerWaypoint = 12;
cfg.visualization.frameDelay = 0.015;
cfg.visualization.figurePosition = [50, 50, 1200, 900];
end
