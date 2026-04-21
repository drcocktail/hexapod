function diagnoseSeed(seed)
%DIAGNOSESEED Mathematically analyse why a seed fails to produce a path.
%
%  diagnoseSeed(8148)   -- reproduce the failing case
%  diagnoseSeed()        -- use a fresh random seed

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'src'));

if nargin < 1 || isempty(seed)
    seed = randi(10000);
end
rng(seed);
fprintf('\n=== SEED %d ===\n\n', seed);

%% 1. Build environment (same logic as runSimulation)
cfg = motionplanning.config.defaultConfig();
robot  = motionplanning.robot.createHexapodRobot(cfg.robot);
terrain = motionplanning.environment.generateVoxelTerrain(cfg.environment);

margin = robot.nominalFootRadius + cfg.environment.endpointMargin;
startNode = [margin, margin];
goalNode  = [terrain.maxX - margin, terrain.maxY - margin];

spawnR = max(cfg.environment.spawnFlatRadius, robot.nominalFootRadius + terrain.gridSpacing);
terrain.Z = motionplanning.environment.flattenRegion(terrain.Z, terrain.X, terrain.Y, startNode, spawnR);
terrain.Z = motionplanning.environment.flattenRegion(terrain.Z, terrain.X, terrain.Y, goalNode,  spawnR);

fprintf('--- 1. Terrain statistics ---\n');
fprintf('  Domain          : %.0f x %.0f  (grid %d x %d, spacing %.3f m)\n', ...
    terrain.maxX, terrain.maxY, terrain.resolution, terrain.resolution, terrain.gridSpacing);
fprintf('  Height range    : [%.2f, %.2f]  (block size %.1f)\n', ...
    min(terrain.Z(:)), max(terrain.Z(:)), cfg.environment.blockHeight);
fprintf('  Max slope proxy : %.2f  (per grid cell)\n', ...
    max(abs(diff(terrain.Z(:)))) / terrain.gridSpacing);

%% 2. Start / goal validity
fprintf('\n--- 2. Start / goal validity ---\n');
initialYaw = atan2(goalNode(2) - startNode(2), goalNode(1) - startNode(1));
[startValid, startZ, startInfo] = motionplanning.planning.isStateValid(startNode, terrain, robot, initialYaw);
[goalValid,  goalZ,  goalInfo ] = motionplanning.planning.isStateValid(goalNode,  terrain, robot, initialYaw);
printStateResult('Start', startNode, startValid, startZ, startInfo);
printStateResult('Goal',  goalNode,  goalValid,  goalZ,  goalInfo);

%% 3. Grid validity survey
fprintf('\n--- 3. Grid validity survey (%d x %d sample grid) ---\n', 20, 20);
surveyN  = 20;
surveyX  = linspace(margin, terrain.maxX - margin, surveyN);
surveyY  = linspace(margin, terrain.maxY - margin, surveyN);
validMap = false(surveyN, surveyN);
reasons  = cell(surveyN, surveyN);
bodyZMap = NaN(surveyN, surveyN);

for xi = 1:surveyN
    for yi = 1:surveyN
        st = [surveyX(xi), surveyY(yi)];
        [v, bz, info] = motionplanning.planning.isStateValid(st, terrain, robot, initialYaw);
        validMap(xi, yi) = v;
        reasons{xi, yi}  = info.reason;
        bodyZMap(xi, yi) = bz;
    end
end

validFraction = mean(validMap(:));
fprintf('  Valid states    : %d / %d  (%.1f%%)\n', sum(validMap(:)), surveyN^2, validFraction * 100);

% Count failure reasons
allReasons = reasons(~validMap);
uniqueReasons = unique(allReasons);
fprintf('  Failure reasons :\n');
for r = 1:numel(uniqueReasons)
    cnt = sum(strcmp(allReasons, uniqueReasons{r}));
    fprintf('    %-40s  %d\n', uniqueReasons{r}, cnt);
end

%% 4. Diagonal corridor profile (start -> goal)
fprintf('\n--- 4. Diagonal corridor profile (start -> goal, 60 samples) ---\n');
nProbe  = 60;
alphas  = linspace(0, 1, nProbe);
diagValid     = false(nProbe, 1);
diagBodyZ     = NaN(nProbe, 1);
diagTerrainZ  = NaN(nProbe, 1);
diagReason    = cell(nProbe, 1);

for k = 1:nProbe
    st = startNode + alphas(k) * (goalNode - startNode);
    tz = motionplanning.environment.getTerrainHeight(st(1), st(2), terrain, 'nan');
    diagTerrainZ(k) = tz;
    [v, bz, info] = motionplanning.planning.isStateValid(st, terrain, robot, initialYaw);
    diagValid(k)  = v;
    diagBodyZ(k)  = bz;
    diagReason{k} = info.reason;
end

validDiag = sum(diagValid);
fprintf('  Valid along diagonal : %d / %d  (%.1f%%)\n', validDiag, nProbe, validDiag / nProbe * 100);

% Find contiguous blocked stretches
blocked   = ~diagValid;
if any(blocked)
    fprintf('  Blocked zones (alpha along diagonal):\n');
    inBlock = false;
    for k = 1:nProbe
        if blocked(k) && ~inBlock
            blockStart = alphas(k);
            inBlock = true;
        end
        if (~blocked(k) || k == nProbe) && inBlock
            blockEnd = alphas(k - 1);
            % Representative point
            midK = round((find(alphas >= blockStart, 1) + find(alphas <= blockEnd, 1, 'last')) / 2);
            midK = min(max(midK, 1), nProbe);
            fprintf('    alpha [%.3f, %.3f]  XY=(%.1f,%.1f)  terrain=%.1f  reason=%s\n', ...
                blockStart, blockEnd, ...
                startNode(1) + blockStart * (goalNode(1) - startNode(1)), ...
                startNode(2) + blockStart * (goalNode(2) - startNode(2)), ...
                diagTerrainZ(midK), diagReason{midK});
            inBlock = false;
        end
    end
end

%% 5. Edge feasibility along diagonal
fprintf('\n--- 5. Edge-to-edge feasibility along diagonal ---\n');
edgeLen    = cfg.planning.stepSize;
nEdges     = floor(norm(goalNode - startNode) / edgeLen);
edgeDir    = (goalNode - startNode) / norm(goalNode - startNode);
edgePts    = startNode + (0:nEdges)' * edgeLen * edgeDir;
edgeValid  = false(nEdges, 1);
edgeReasons = cell(nEdges, 1);

for k = 1:nEdges
    [ev, ~, ~, fr] = motionplanning.planning.isEdgeValid( ...
        edgePts(k,:), edgePts(k+1,:), terrain, robot, cfg.planning);
    edgeValid(k)   = ev;
    edgeReasons{k} = fr;
end

validEdges = sum(edgeValid);
fprintf('  Step size       : %.1f m  |  Total edges : %d\n', edgeLen, nEdges);
fprintf('  Valid edges     : %d / %d  (%.1f%%)\n', validEdges, nEdges, validEdges / nEdges * 100);
firstFail = find(~edgeValid, 1);
if ~isempty(firstFail)
    fprintf('  First failure   : edge %d (at XY=%.1f,%.1f)  reason=%s\n', ...
        firstFail, edgePts(firstFail,1), edgePts(firstFail,2), edgeReasons{firstFail});
end

%% 6. maxStepZ analysis along diagonal
fprintf('\n--- 6. Body-Z delta analysis (maxStepZ=%.1f, resolution=%.1f) ---\n', ...
    robot.maxStepZ, cfg.planning.edgeCheckResolution);

validBodyZ = diagBodyZ(diagValid);
if numel(validBodyZ) > 1
    bodyZDeltas = abs(diff(validBodyZ));
    fprintf('  Max |ΔbodyZ| between consecutive valid diagonal samples : %.3f\n', max(bodyZDeltas));
    fprintf('  Mean |ΔbodyZ|                                           : %.3f\n', mean(bodyZDeltas));
    nExceed = sum(bodyZDeltas > robot.maxStepZ);
    fprintf('  Samples where |ΔbodyZ| > maxStepZ                       : %d\n', nExceed);
end

% Terrain height change per grid cell (worst-case slope)
dZdX = abs(diff(terrain.Z, 1, 2)) / terrain.gridSpacing;
dZdY = abs(diff(terrain.Z, 1, 1)) / terrain.gridSpacing;
fprintf('  Terrain max |dZ/dx|                                      : %.3f  (limit ~%.3f)\n', ...
    max(dZdX(:)), robot.maxStepZ / cfg.planning.edgeCheckResolution);

%% 7. Connectivity summary
fprintf('\n--- 7. Reachability summary ---\n');
fprintf('  Planner budget   : %d nodes  step %.1f m  goal-bias %.0f%%\n', ...
    cfg.planning.maxNodes, cfg.planning.stepSize, cfg.planning.goalBias * 100);

straightDist = norm(goalNode - startNode);
fprintf('  Straight-line XY : %.1f m  (~%.0f steps)\n', straightDist, straightDist / cfg.planning.stepSize);

if validFraction < 0.4
    fprintf('  ** WARNING: only %.0f%% of sampled states are valid.\n', validFraction * 100);
    fprintf('  ** The terrain may create a barrier with no navigable corridor.\n');
elseif validDiag < nProbe * 0.5
    fprintf('  ** WARNING: diagonal corridor is blocked at %.0f%% of samples.\n', ...
        (1 - validDiag / nProbe) * 100);
    fprintf('  ** A detour is needed; 6000 nodes may be insufficient.\n');
else
    fprintf('  ** Terrain looks navigable; failure is likely stochastic.\n');
    fprintf('  ** Try: main(''maxnodes'', 12000) or a different seed.\n');
end

fprintf('\n');
end

function printStateResult(label, st, isValid, bodyZ, info)
if isValid
    fprintf('  %-6s XY=(%.2f,%.2f)  bodyZ=%.3f  VALID\n', label, st(1), st(2), bodyZ);
else
    fprintf('  %-6s XY=(%.2f,%.2f)  INVALID  reason=%s', label, st(1), st(2), info.reason);
    if ~isnan(info.pitch)
        fprintf('  pitch=%.1f° roll=%.1f°', rad2deg(info.pitch), rad2deg(info.roll));
    end
    fprintf('\n');
end
end
