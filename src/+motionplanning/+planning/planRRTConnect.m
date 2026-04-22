function [path, pathZ, info] = planRRTConnect(startState, goalState, terrain, robot, options)
%PLANRRTCONNECT Bidirectional greedy RRT for local segment repair.
%
%   Grows two trees simultaneously from startState and goalState,
%   alternating extensions.  Uses greedy CONNECT (repeated stepping)
%   toward the other tree's latest node.  No cost tracking, no rewiring
%   — pure speed to connection.
%
%   Typically invoked by validateAndRepairPath on small local segments,
%   not on the full domain.

stepSize  = getOpt(options, 'rrtConnectStepSize', 3.0);
maxIter   = getOpt(options, 'rrtConnectMaxIter', 500);
edgeRes   = getOpt(options, 'edgeCheckResolution', 2.0);

info = struct( ...
    'status', 'not_started', ...
    'iterations', 0, ...
    'failureReason', '', ...
    'treeASize', 0, ...
    'treeBSize', 0);

% Compute endpoint heights.  The caller (validateAndRepairPath) has
% already verified these endpoints pass isStateValid with the correct
% yaw.  Re-validating here with a local yaw would cause false rejections
% when the yaw differs (isStateValid is yaw-sensitive due to foot
% placement and body orientation).
initialYaw = atan2(goalState(2) - startState(2), goalState(1) - startState(1));
[startValid, startZ] = motionplanning.planning.isStateValid(startState, terrain, robot, initialYaw);
[goalValid,  goalZ]  = motionplanning.planning.isStateValid(goalState,  terrain, robot, initialYaw);

if ~startValid
    % Retry with cardinal yaw (0) as fallback — the endpoint is known
    % valid from the caller's check with a different yaw.
    [startValid, startZ] = motionplanning.planning.isStateValid(startState, terrain, robot, 0);
end
if ~goalValid
    [goalValid, goalZ] = motionplanning.planning.isStateValid(goalState, terrain, robot, 0);
end

% If still invalid, compute Z from terrain directly and proceed anyway.
% The caller has vouched for these points.
if ~startValid
    tz = motionplanning.environment.getTerrainHeight( ...
        startState(1), startState(2), terrain, 'clamp');
    startZ = tz + robot.clearance + robot.bodyHeight / 2;
end
if ~goalValid
    tz = motionplanning.environment.getTerrainHeight( ...
        goalState(1), goalState(2), terrain, 'clamp');
    goalZ = tz + robot.clearance + robot.bodyHeight / 2;
end

% Local sampling bounds: bounding box of segment ± padding
repairDist = norm(goalState - startState);
padding = max([robot.bodyRadius * 5, stepSize * 6, repairDist * 0.5]);
bbMin = min(startState, goalState) - padding;
bbMax = max(startState, goalState) + padding;
bbMin = max(bbMin, [terrain.minX, terrain.minY]);
bbMax = min(bbMax, [terrain.maxX, terrain.maxY]);

% --- Tree A (forward from start) ---
capA = maxIter + 2;
nodesA   = zeros(capA, 2);
nodesZA  = NaN(capA, 1);
parentsA = zeros(capA, 1);
nodesA(1, :) = startState;
nodesZA(1)   = startZ;
sizeA = 1;

% --- Tree B (backward from goal) ---
capB = maxIter + 2;
nodesB   = zeros(capB, 2);
nodesZB  = NaN(capB, 1);
parentsB = zeros(capB, 1);
nodesB(1, :) = goalState;
nodesZB(1)   = goalZ;
sizeB = 1;

connected = false;
connA = 0;
connB = 0;

for iter = 1:maxIter
    info.iterations = iter;

    % --- Sample random state in local bounding box ---
    randState = bbMin + rand(1, 2) .* (bbMax - bbMin);

    % --- EXTEND tree A toward random sample ---
    [sizeA, nodesA, nodesZA, parentsA, newIdxA, extendedA] = ...
        extendTree(sizeA, nodesA, nodesZA, parentsA, randState, ...
        stepSize, terrain, robot, options);

    if ~extendedA
        % Swap and try again next iteration
        [nodesA, nodesB] = deal(nodesB, nodesA);
        [nodesZA, nodesZB] = deal(nodesZB, nodesZA);
        [parentsA, parentsB] = deal(parentsB, parentsA);
        [sizeA, sizeB] = deal(sizeB, sizeA);
        continue;
    end

    % --- CONNECT tree B greedily toward tree A's new node ---
    targetState = nodesA(newIdxA, :);
    [sizeB, nodesB, nodesZB, parentsB, reachedB] = ...
        connectTree(sizeB, nodesB, nodesZB, parentsB, targetState, ...
        stepSize, terrain, robot, options);

    if reachedB
        connected = true;
        connA = newIdxA;
        connB = sizeB;
        break;
    end

    % Swap trees for next iteration (alternate growth direction)
    [nodesA, nodesB] = deal(nodesB, nodesA);
    [nodesZA, nodesZB] = deal(nodesZB, nodesZA);
    [parentsA, parentsB] = deal(parentsB, parentsA);
    [sizeA, sizeB] = deal(sizeB, sizeA);
    [capA, capB] = deal(capB, capA); %#ok<ASGLU>
end

info.treeASize = sizeA;
info.treeBSize = sizeB;

if ~connected
    path = []; pathZ = [];
    info.status = 'rrt_connect_failed';
    info.failureReason = 'max_iterations_exhausted';
    return;
end

% --- Reconstruct path through both trees ---
% After swaps, we need to figure out which tree is forward vs backward.
% connA is an index into nodesA, connB into nodesB.
% Tree A's root is either startState or goalState.

pathA_XY = traceBack(nodesA, parentsA, connA);
pathA_Z  = traceBackZ(nodesZA, parentsA, connA);
pathB_XY = traceBack(nodesB, parentsB, connB);
pathB_Z  = traceBackZ(nodesZB, parentsB, connB);

% Determine which tree starts at startState
if norm(nodesA(1, :) - startState) < eps
    % A is forward, B is backward
    pathA_XY = flipud(pathA_XY);   % root -> connA
    pathA_Z  = flipud(pathA_Z);
    % pathB already goes connB -> root(goal)
    path  = [pathA_XY; pathB_XY(2:end, :)];  % skip duplicate connection point
    pathZ = [pathA_Z;  pathB_Z(2:end)];
else
    % A is backward (goal->connA), B is forward (start->connB)
    pathB_XY = flipud(pathB_XY);   % root(start) -> connB
    pathB_Z  = flipud(pathB_Z);
    path  = [pathB_XY; pathA_XY(2:end, :)];
    pathZ = [pathB_Z;  pathA_Z(2:end)];
end

info.status = 'goal_reached';
end

%% ---- Local helpers ----

function [treeSize, nodes, nodesZ, parents, newIdx, success] = ...
    extendTree(treeSize, nodes, nodesZ, parents, target, stepSize, terrain, robot, options)
%EXTENDTREE Single-step extension toward target.

success = false;
newIdx  = 0;

% Find nearest node in tree
nearestIdx = findNearest(nodes, treeSize, target);
nearestNode = nodes(nearestIdx, :);

% Steer toward target
newState = motionplanning.planning.steer(nearestNode, target, stepSize);
if norm(newState - nearestNode) < eps
    return;
end

% Validate edge
[edgeValid, zSamples, ~, ~] = motionplanning.planning.isEdgeValid( ...
    nearestNode, newState, terrain, robot, options);
if ~edgeValid
    return;
end

% Add node
if treeSize + 1 > size(nodes, 1)
    return;  % capacity exhausted
end
treeSize = treeSize + 1;
newIdx = treeSize;
nodes(newIdx, :)  = newState;
nodesZ(newIdx)    = zSamples(end);
parents(newIdx)   = nearestIdx;
success = true;
end

function [treeSize, nodes, nodesZ, parents, reached] = ...
    connectTree(treeSize, nodes, nodesZ, parents, target, stepSize, terrain, robot, options)
%CONNECTTREE Greedily extend tree toward target until blocked or reached.

reached = false;
maxAttempts = 50;  % prevent infinite greedy loop

for attempt = 1:maxAttempts %#ok<NASGU>
    nearestIdx = findNearest(nodes, treeSize, target);
    nearestNode = nodes(nearestIdx, :);

    dist = norm(target - nearestNode);
    if dist < stepSize * 0.5
        reached = true;
        return;
    end

    newState = motionplanning.planning.steer(nearestNode, target, stepSize);
    if norm(newState - nearestNode) < eps
        return;
    end

    [edgeValid, zSamples, ~, ~] = motionplanning.planning.isEdgeValid( ...
        nearestNode, newState, terrain, robot, options);
    if ~edgeValid
        return;  % blocked, stop connecting
    end

    if treeSize + 1 > size(nodes, 1)
        return;
    end
    treeSize = treeSize + 1;
    nodes(treeSize, :) = newState;
    nodesZ(treeSize)   = zSamples(end);
    parents(treeSize)  = nearestIdx;

    if norm(newState - target) < stepSize * 0.5
        reached = true;
        return;
    end
end
end

function nearestIdx = findNearest(nodes, treeSize, query)
distsSq = sum((nodes(1:treeSize, :) - query) .^ 2, 2);
[~, nearestIdx] = min(distsSq);
end

function pathXY = traceBack(nodes, parents, idx)
% Trace from idx back to root (returns in reverse: idx -> root)
count = 0;
current = idx;
while current > 0
    count = count + 1;
    current = parents(current);
end
pathXY = zeros(count, 2);
current = idx;
for k = 1:count
    pathXY(k, :) = nodes(current, :);
    current = parents(current);
end
end

function z = traceBackZ(nodesZ, parents, idx)
count = 0;
current = idx;
while current > 0
    count = count + 1;
    current = parents(current);
end
z = zeros(count, 1);
current = idx;
for k = 1:count
    z(k) = nodesZ(current);
    current = parents(current);
end
end

function value = getOpt(options, name, default)
if isfield(options, name)
    value = options.(name);
else
    value = default;
end
end
