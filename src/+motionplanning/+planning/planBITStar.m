function [path, pathZ, info] = planBITStar(startState, goalState, terrain, robot, options)
%PLANBITSTAR Batch-informed tree planner with BIT*-style edge ordering.

maxNodes = options.bitMaxBatches * options.bitBatchSize + 2;
nodes = zeros(maxNodes, 2);
nodesZ = NaN(maxNodes, 1);
parents = zeros(maxNodes, 1);
costs = Inf(maxNodes, 1);
edgeCosts = zeros(maxNodes, 1);

info = struct( ...
    'status', 'not_started', ...
    'iterations', 0, ...
    'nodeCount', 0, ...
    'failureReason', '', ...
    'startValid', false, ...
    'goalValid', false, ...
    'bestCost', Inf, ...
    'batches', 0);

initialYaw = atan2(goalState(2) - startState(2), goalState(1) - startState(1));
[startValid, startZ, startInfo] = motionplanning.planning.isStateValid(startState, terrain, robot, initialYaw);
[goalValid, goalZ, goalInfo] = motionplanning.planning.isStateValid(goalState, terrain, robot, initialYaw);

info.startValid = startValid;
info.goalValid = goalValid;
if ~startValid
    [path, pathZ, info] = invalidResult(info, 'invalid_start', startInfo.reason);
    return;
end
if ~goalValid
    [path, pathZ, info] = invalidResult(info, 'invalid_goal', goalInfo.reason);
    return;
end

nodes(1, :) = startState;
nodesZ(1) = startZ;
costs(1) = 0;
nodeCount = 1;
goalIdx = NaN;
bestCost = Inf;
samples = zeros(0, 2);

[goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, info] = ...
    tryConnectGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
    goalState, goalZ, terrain, robot, options, info);

for batch = 1:options.bitMaxBatches
    info.batches = batch;
    info.iterations = batch;

    samples = [samples; drawBatch(startState, goalState, bestCost, terrain, options)]; %#ok<AGROW>
    samples = pruneSamples(samples, goalState, bestCost);
    if isempty(samples)
        continue;
    end

    candidateEdges = buildCandidateEdges(nodes, costs, nodeCount, samples, goalState, bestCost, options);
    if isempty(candidateEdges)
        continue;
    end

    candidateEdges = sortrows(candidateEdges, 1);
    sampleActive = true(size(samples, 1), 1);

    for edgeIdx = 1:size(candidateEdges, 1)
        lowerBound = candidateEdges(edgeIdx, 1);
        if lowerBound >= bestCost
            break;
        end

        parentIdx = candidateEdges(edgeIdx, 2);
        sampleIdx = candidateEdges(edgeIdx, 3);
        if ~sampleActive(sampleIdx)
            continue;
        end

        sampleState = samples(sampleIdx, :);
        [candidateEdgeCost, candidateZ, edgeValid, failReason] = motionplanning.planning.edgeCost( ...
            nodes(parentIdx, :), sampleState, terrain, robot, options);
        if ~edgeValid
            info.failureReason = failReason;
            continue;
        end

        candidateCost = costs(parentIdx) + candidateEdgeCost;
        if candidateCost + motionplanning.planning.heuristicCost(sampleState, goalState) >= bestCost
            continue;
        end

        if nodeCount + 1 > maxNodes
            info.status = 'capacity_exhausted';
            break;
        end

        nodeCount = nodeCount + 1;
        nodes(nodeCount, :) = sampleState;
        nodesZ(nodeCount) = candidateZ;
        parents(nodeCount) = parentIdx;
        costs(nodeCount) = candidateCost;
        edgeCosts(nodeCount) = candidateEdgeCost;
        sampleActive(sampleIdx) = false;

        [goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, info] = ...
            tryConnectGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
            goalState, goalZ, terrain, robot, options, info);
    end

    samples = samples(sampleActive, :);
    [nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx] = ...
        pruneTree(nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx, goalState, bestCost);

    if strcmp(info.status, 'capacity_exhausted')
        break;
    end
end

info.nodeCount = nodeCount;
info.bestCost = bestCost;
if isnan(goalIdx)
    path = [];
    pathZ = [];
    if strcmp(info.status, 'not_started')
        info.status = 'max_batches_exhausted';
    end
    return;
end

[path, pathZ] = motionplanning.planning.reconstructPath(nodes, nodesZ, parents, goalIdx);
pathZ(end) = goalZ;
info.status = 'goal_reached';
end

function [path, pathZ, info] = invalidResult(info, status, reason)
path = [];
pathZ = [];
info.status = status;
info.failureReason = reason;
end

function samples = drawBatch(startState, goalState, bestCost, terrain, options)
samples = zeros(options.bitBatchSize, 2);
for idx = 1:options.bitBatchSize
    samples(idx, :) = motionplanning.planning.sampleInformedState( ...
        startState, goalState, bestCost, terrain, options);
end
end

function samples = pruneSamples(samples, goalState, bestCost)
if ~isfinite(bestCost) || isempty(samples)
    return;
end
dists   = sqrt(sum((samples - goalState) .^ 2, 2));
samples = samples(dists < bestCost, :);
end

function candidateEdges = buildCandidateEdges(nodes, costs, nodeCount, samples, goalState, bestCost, options)
radius   = options.bitConnectionRadius;
nSamples = size(samples, 1);

% Pre-allocate worst-case; trim at the end.
maxEdges = nodeCount * nSamples;
edgeLB   = zeros(maxEdges, 1);
edgeFrom = zeros(maxEdges, 1);
edgeTo   = zeros(maxEdges, 1);
edgeCount = 0;

% Cache per-sample goal distances to avoid recomputing inside the vertex loop.
goalDists = sqrt(sum((samples - goalState) .^ 2, 2));

for vertexIdx = 1:nodeCount
    if ~isfinite(costs(vertexIdx))
        continue;
    end

    deltas    = samples - nodes(vertexIdx, :);
    distances = sqrt(sum(deltas .^ 2, 2));
    nearMask  = distances <= radius;
    if ~any(nearMask)
        continue;
    end

    lbs      = costs(vertexIdx) + distances(nearMask) + goalDists(nearMask);
    validMask = lbs < bestCost;
    nearIdx   = find(nearMask);
    validIdx  = nearIdx(validMask);
    n         = numel(validIdx);
    if n > 0
        edgeLB(edgeCount + 1 : edgeCount + n)   = lbs(validMask);
        edgeFrom(edgeCount + 1 : edgeCount + n) = vertexIdx;
        edgeTo(edgeCount + 1 : edgeCount + n)   = validIdx;
        edgeCount = edgeCount + n;
    end
end

candidateEdges = [edgeLB(1:edgeCount), edgeFrom(1:edgeCount), edgeTo(1:edgeCount)];
end

function [goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, info] = ...
    tryConnectGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
    goalState, goalZ, terrain, robot, options, info)

newIdx = nodeCount;
lowerBound = costs(newIdx) + motionplanning.planning.heuristicCost(nodes(newIdx, :), goalState);
if lowerBound >= bestCost
    return;
end

if norm(nodes(newIdx, :) - goalState) > options.bitConnectionRadius
    return;
end

[goalEdgeCost, goalEdgeZ, edgeValid, failReason] = motionplanning.planning.edgeCost( ...
    nodes(newIdx, :), goalState, terrain, robot, options);
if ~edgeValid
    info.failureReason = failReason;
    return;
end

candidateCost = costs(newIdx) + goalEdgeCost;
if candidateCost >= bestCost
    return;
end

if isnan(goalIdx)
    if nodeCount + 1 > size(nodes, 1)
        info.status = 'capacity_exhausted';
        return;
    end
    nodeCount = nodeCount + 1;
    goalIdx = nodeCount;
    nodes(goalIdx, :) = goalState;
    nodesZ(goalIdx) = goalEdgeZ;
else
    nodesZ(goalIdx) = goalZ;
end

parents(goalIdx) = newIdx;
costs(goalIdx) = candidateCost;
edgeCosts(goalIdx) = goalEdgeCost;
bestCost = candidateCost;
info.status = 'goal_reached';
info.bestCost = bestCost;
end

function [nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx] = ...
    pruneTree(nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx, goalState, bestCost)

if ~isfinite(bestCost)
    return;
end

keep = true(nodeCount, 1);
for idx = 2:nodeCount
    if idx == goalIdx
        continue;
    end
    keep(idx) = costs(idx) + motionplanning.planning.heuristicCost(nodes(idx, :), goalState) < bestCost;
end

if ~isnan(goalIdx)
    current = goalIdx;
    while current > 0
        keep(current) = true;
        current = parents(current);
    end
end

% Propagate cost-pruning down the tree: a child of a pruned node must also
% be pruned.  Build a children list once (O(n)) then BFS from the root so
% the whole pass is O(n) instead of the O(n²) while-changed loop.
childLists = cell(nodeCount, 1);
for idx = 2:nodeCount
    p = parents(idx);
    if p >= 1 && p <= nodeCount
        childLists{p}(end + 1) = idx;
    end
end
reachable    = false(nodeCount, 1);
bfsQueue     = zeros(nodeCount, 1);
bfsQueue(1)  = 1;
bfsHead      = 1;
bfsTail      = 1;
while bfsHead <= bfsTail
    curr    = bfsQueue(bfsHead);
    bfsHead = bfsHead + 1;
    if ~keep(curr)
        continue;
    end
    reachable(curr) = true;
    for c = childLists{curr}
        bfsTail             = bfsTail + 1;
        bfsQueue(bfsTail)   = c;
    end
end
keep = keep & reachable;

oldToNew = zeros(nodeCount, 1);
newIdx = find(keep);
oldToNew(newIdx) = 1:numel(newIdx);

nodes(1:numel(newIdx), :) = nodes(newIdx, :);
nodesZ(1:numel(newIdx)) = nodesZ(newIdx);
costs(1:numel(newIdx)) = costs(newIdx);
edgeCosts(1:numel(newIdx)) = edgeCosts(newIdx);

newParents = zeros(numel(newIdx), 1);
for idx = 1:numel(newIdx)
    oldParent = parents(newIdx(idx));
    if oldParent > 0
        newParents(idx) = oldToNew(oldParent);
    end
end
parents(1:numel(newIdx)) = newParents;

if isnan(goalIdx)
    goalIdx = NaN;
else
    goalIdx = oldToNew(goalIdx);
    if goalIdx == 0
        goalIdx = NaN;
    end
end

nodeCount = numel(newIdx);
end
