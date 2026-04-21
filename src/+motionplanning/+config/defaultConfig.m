function cfg = defaultConfig()
%DEFAULTCONFIG Return simulator defaults in one editable structure.

cfg.seed = [];
cfg.seedRange = 10000;

cfg.robot.bodyRadius = 4.0;
cfg.robot.bodyHeight = 2.0;
cfg.robot.clearance = 4.0;
cfg.robot.L_coxa = 1.5;
cfg.robot.L_femur = 4.5;
cfg.robot.L_tibia = 5.5;
cfg.robot.maxStepZ = 10.0;
cfg.robot.maxPitch = 45 * pi / 180;
cfg.robot.maxRoll = 45 * pi / 180;
cfg.robot.stabilityMargin = 0.25;

cfg.environment.gridResolution = 220;
cfg.environment.domainSize = 180;
cfg.environment.octaves = 6;
cfg.environment.persistence = 0.55;
cfg.environment.frequency = 0.012;
cfg.environment.amplitude = 18.0;
cfg.environment.blockHeight = 2.0;
cfg.environment.topologyFeatureCount = 70;
cfg.environment.topologyFeatureHeight = 16.0;
cfg.environment.topologyMinRadius = 5.0;
cfg.environment.topologyMaxRadius = 28.0;
cfg.environment.faultStepHeight = 10.0;
cfg.environment.maxTerrainHeight = 38.0;
cfg.environment.diagonalValleyDepth = 0.0;
cfg.environment.valleyInfluence = 6.0;
cfg.environment.valleyDepth = 0.0;
cfg.environment.endpointMargin = 2.0;
cfg.environment.spawnFlatRadius = cfg.robot.bodyRadius * 1.5;

cfg.start = [];
cfg.goal = [];

cfg.planning.algorithm = 'informed_rrt_star';
cfg.planning.maxNodes = 6000;
cfg.planning.stepSize = 5.0;
cfg.planning.goalBias = 0.12;
cfg.planning.goalTolerance = cfg.planning.stepSize;
cfg.planning.edgeCheckResolution = 2.0;
cfg.planning.nearestMode = 'spatial_hash';
cfg.planning.spatialCellSize = cfg.planning.stepSize * 4;
cfg.planning.rewireRadius = cfg.planning.stepSize * 6;
cfg.planning.rrtStarGamma = cfg.environment.domainSize * 1.5;
cfg.planning.solutionCheckInterval = 10;
cfg.planning.distanceWeight = 1.0;
cfg.planning.verticalWeight = 0.25;
cfg.planning.roughnessWeight = 0.1;
cfg.planning.bitBatchSize = 220;
cfg.planning.bitMaxBatches = 35;
cfg.planning.bitConnectionRadius = cfg.planning.stepSize * 7;

cfg.visualization.enabled = true;
cfg.visualization.animate = true;
cfg.visualization.framesPerWaypoint = 6;
cfg.visualization.frameDelay = 0.005;
cfg.visualization.figurePosition = [50, 50, 1200, 900];
end
