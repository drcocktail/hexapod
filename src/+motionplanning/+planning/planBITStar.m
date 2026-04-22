function [path, pathZ, info] = planBITStar(startState, goalState, terrain, robot, options)
%PLANBITSTAR Batch Informed Trees (BIT*) planner with lazy evaluation,
%   deferred sample integration, and vertex-to-vertex rewiring.
%
%   This implementation follows the formal BIT* algorithm:
%     1. Draw informed samples and build candidate edges (vectorized).
%     2. Evaluate edges lazily — skip expensive IK when a theoretical
%        lower bound proves the edge cannot improve the sample's best
%        known cost.
%     3. Defer tree insertion: samples are grafted to their optimal
%        parent only after the entire edge queue is exhausted.
%     4. Rewire existing tree vertices through newly added nodes.

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

fprintf('BIT* Solver Started. Goal is to connect across %d max batches.\n', options.bitMaxBatches);
lastLogTime = tic;
stagnationBatches = 0;
previousBestCost = Inf;

for batch = 1:options.bitMaxBatches
    info.batches = batch;
    info.iterations = batch;
    batchEdgesTested = 0;
    batchFails = 0;
    batchLazySkips = 0;

    samples = [samples; drawBatch(startState, goalState, bestCost, terrain, options)]; %#ok<AGROW>
    samples = pruneSamples(samples, goalState, bestCost);
    if isempty(samples)
        continue;
    end

    nSamples = size(samples, 1);

    % --- Optimization C: Vectorized edge building via pdist2 ---
    candidateEdges = buildCandidateEdges(nodes, costs, nodeCount, samples, goalState, bestCost, options);
    if isempty(candidateEdges)
        continue;
    end

    candidateEdges = sortrows(candidateEdges, 1);

    if toc(lastLogTime) > 3.0 || batch == 1
        fprintf('  [BIT*] Batch %d/%d: %d samples -> %d candidate edges. Tree: %d nodes | Best: %.2f\n', ...
            batch, options.bitMaxBatches, nSamples, size(candidateEdges, 1), nodeCount, bestCost);
        lastLogTime = tic;
    end

    % --- Optimization A: Lazy cost bounds with deferred integration ---
    % Track the best validated connection for each sample across the
    % entire edge queue.  No sample is locked on first contact.
    sampleBestCost     = Inf(nSamples, 1);
    sampleBestParent   = zeros(nSamples, 1);
    sampleBestZ        = NaN(nSamples, 1);
    sampleBestEdgeCost = Inf(nSamples, 1);

    for edgeIdx = 1:size(candidateEdges, 1)
        lowerBound = candidateEdges(edgeIdx, 1);
        if lowerBound >= bestCost
            break;
        end

        parentIdx = candidateEdges(edgeIdx, 2);
        sampleIdx = candidateEdges(edgeIdx, 3);

        % Lazy rejection: if the theoretical best cost through this parent
        % cannot beat the sample's current best validated cost, skip the
        % expensive edgeCost call entirely.
        euclidDist = norm(samples(sampleIdx, :) - nodes(parentIdx, :));
        candidateTheoreticalCost = costs(parentIdx) + euclidDist;
        if candidateTheoreticalCost >= sampleBestCost(sampleIdx)
            batchLazySkips = batchLazySkips + 1;
            continue;
        end

        batchEdgesTested = batchEdgesTested + 1;

        [candidateEdgeCost, candidateZ, edgeValid, failReason] = motionplanning.planning.edgeCost( ...
            nodes(parentIdx, :), samples(sampleIdx, :), terrain, robot, options);
        if ~edgeValid
            info.failureReason = failReason;
            batchFails = batchFails + 1;
            continue;
        end

        candidateCost = costs(parentIdx) + candidateEdgeCost;

        % Global admissibility check: can this sample improve the incumbent?
        if candidateCost + motionplanning.planning.heuristicCost(samples(sampleIdx, :), goalState) >= bestCost
            continue;
        end

        % Update the sample's best connection if this edge is strictly better.
        if candidateCost < sampleBestCost(sampleIdx)
            sampleBestCost(sampleIdx)     = candidateCost;
            sampleBestParent(sampleIdx)   = parentIdx;
            sampleBestZ(sampleIdx)        = candidateZ;
            sampleBestEdgeCost(sampleIdx) = candidateEdgeCost;
        end
    end

    % --- Deferred integration phase ---
    % Commit only the optimal parent discovered for each sample.
    integratedMask = isfinite(sampleBestCost);
    integratedIdx  = find(integratedMask);
    newNodeIndices = zeros(numel(integratedIdx), 1);  % track for rewiring

    for k = 1:numel(integratedIdx)
        si = integratedIdx(k);
        if nodeCount + 1 > maxNodes
            info.status = 'capacity_exhausted';
            break;
        end

        nodeCount = nodeCount + 1;
        nodes(nodeCount, :)  = samples(si, :);
        nodesZ(nodeCount)    = sampleBestZ(si);
        parents(nodeCount)   = sampleBestParent(si);
        costs(nodeCount)     = sampleBestCost(si);
        edgeCosts(nodeCount) = sampleBestEdgeCost(si);
        newNodeIndices(k)    = nodeCount;

        [goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, info] = ...
            tryConnectGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
            goalState, goalZ, terrain, robot, options, info);
    end
    newNodeIndices = newNodeIndices(newNodeIndices > 0);

    % --- Optimization B: Vertex-to-vertex rewiring ---
    batchRewires = 0;
    if ~isempty(newNodeIndices)
        [parents, costs, edgeCosts, batchRewires, goalIdx, bestCost, info] = ...
            rewireFromNewNodes(newNodeIndices, nodes, nodesZ, parents, costs, edgeCosts, ...
            nodeCount, goalIdx, bestCost, terrain, robot, options, info);
    end

    % Remove integrated samples from the pool; keep unintegrated ones.
    samples = samples(~integratedMask, :);

    [nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx] = ...
        pruneTree(nodes, nodesZ, parents, costs, edgeCosts, nodeCount, goalIdx, goalState, bestCost);

    if strcmp(info.status, 'capacity_exhausted')
        fprintf('  [BIT*] Warning: Internal node capacity exhausted at %d nodes.\n', nodeCount);
        break;
    end

    if batchEdgesTested > 0
        fprintf('    -> Batch %d: %d edges evaluated, %d IK-rejected, %d lazy-skipped, %d integrated, %d rewires. Tree: %d | Cost: %.2f\n', ...
            batch, batchEdgesTested, batchFails, batchLazySkips, numel(newNodeIndices), batchRewires, nodeCount, bestCost);
    end

    stagnationTolerance = 5.0;
    if isfield(options, 'bitStagnationTolerance')
        stagnationTolerance = options.bitStagnationTolerance;
    end
    stagnationLimit = 3;
    if isfield(options, 'bitStagnationLimit')
        stagnationLimit = options.bitStagnationLimit;
    end

    if isfinite(bestCost)
        if abs(previousBestCost - bestCost) <= stagnationTolerance
            stagnationBatches = stagnationBatches + 1;
            if stagnationBatches >= stagnationLimit
                fprintf('  [BIT*] Early stopping triggered! Path cost stagnated around %.2f for %d batches.\n', bestCost, stagnationBatches);
                break;
            end
        else
            stagnationBatches = 0;
        end
    end
    previousBestCost = bestCost;
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

%% ---- Local helper functions ----

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
%BUILDCANDIDATEEDGES Vectorized edge generation using pdist2.
%   Returns an Nx3 matrix [lowerBound, vertexIdx, sampleIdx] sorted-ready.

radius = options.bitConnectionRadius;

% Compute all goal distances once (vector).
goalDists = sqrt(sum((samples - goalState) .^ 2, 2));

% Vectorized pairwise distance computation — executed in optimized
% C/BLAS layer, orders of magnitude faster than the per-vertex loop.
activeNodes = nodes(1:nodeCount, :);
activeCosts = costs(1:nodeCount);

% Only consider vertices with finite cost.
finiteMask = isfinite(activeCosts);
finiteIdx  = find(finiteMask);
if isempty(finiteIdx)
    candidateEdges = [];
    return;
end

% Manual vectorized Euclidean distance (no Statistics Toolbox required).
% Uses the identity: ||a-b||^2 = ||a||^2 + ||b||^2 - 2*a*b'
A = activeNodes(finiteIdx, :);
sumA2 = sum(A .^ 2, 2);      % [nVertices x 1]
sumB2 = sum(samples .^ 2, 2); % [nSamples x 1]
distances = sqrt(max(bsxfun(@plus, sumA2, sumB2') - 2 * (A * samples'), 0));

% Find all (vertex, sample) pairs within the connection radius.
[localVertexIdx, sampleIdx] = find(distances <= radius);
if isempty(localVertexIdx)
    candidateEdges = [];
    return;
end

% Map local indices back to global vertex indices.
% Force column vectors throughout to prevent implicit expansion from
% broadcasting mismatched orientations into a matrix.
vertexIdx = finiteIdx(localVertexIdx(:));
sampleIdx = sampleIdx(:);

% Extract corresponding distances using linear indexing.
linearInd = sub2ind(size(distances), localVertexIdx(:), sampleIdx);
distVals  = distances(linearInd);

% Compute lower bounds: cost-to-vertex + edge-distance + heuristic-to-goal.
lbs = activeCosts(vertexIdx) + distVals(:) + goalDists(sampleIdx);

% Filter edges whose lower bound already exceeds the incumbent best cost.
improveMask = lbs < bestCost;
if ~any(improveMask)
    candidateEdges = [];
    return;
end

candidateEdges = [lbs(improveMask), vertexIdx(improveMask), sampleIdx(improveMask)];
end

function [parents, costs, edgeCosts, totalRewires, goalIdx, bestCost, info] = ...
    rewireFromNewNodes(newNodeIndices, nodes, nodesZ, parents, costs, edgeCosts, ...
    nodeCount, goalIdx, bestCost, terrain, robot, options, info)
%REWIREFROMNEWNODES Attempt to rewire existing tree vertices through newly
%   added nodes.  This is the key phase that makes BIT* asymptotically
%   optimal — without it, suboptimal early connections propagate forever.

radius = options.bitConnectionRadius;
totalRewires = 0;

for ni = 1:numel(newNodeIndices)
    vNew = newNodeIndices(ni);
    newNode = nodes(vNew, :);

    % Find existing tree vertices near v_new (vectorized distance check).
    deltas    = nodes(1:nodeCount, :) - newNode;
    distsSq   = sum(deltas .^ 2, 2);
    nearMask  = distsSq <= radius^2;

    % Exclude root, self, and the current parent of v_new.
    nearMask(1)    = false;
    nearMask(vNew) = false;
    if parents(vNew) > 0 && parents(vNew) <= nodeCount
        nearMask(parents(vNew)) = false;
    end
    % Exclude the goal node from rewiring targets (it is managed separately).
    if ~isnan(goalIdx) && goalIdx <= nodeCount
        nearMask(goalIdx) = false;
    end

    neighbors = find(nearMask);
    if isempty(neighbors)
        continue;
    end

    dists = sqrt(distsSq(neighbors));

    % Vectorized lower-bound pre-filter.
    lbs = costs(vNew) + dists;
    passMask = lbs < costs(neighbors);
    neighbors = neighbors(passMask);
    dists = dists(passMask); %#ok<NASGU>

    for neighborIdx = neighbors(:)'
        % Ancestor check: prevent cycles.
        if isAncestor(neighborIdx, vNew, parents)
            continue;
        end

        [candidateEdgeCost, ~, edgeValid] = motionplanning.planning.edgeCost( ...
            nodes(vNew, :), nodes(neighborIdx, :), terrain, robot, options);
        if ~edgeValid
            continue;
        end

        candidateCost = costs(vNew) + candidateEdgeCost;
        if candidateCost < costs(neighborIdx)
            parents(neighborIdx)   = vNew;
            edgeCosts(neighborIdx) = candidateEdgeCost;
            costs(neighborIdx)     = candidateCost;
            costs = motionplanning.planning.propagateCosts( ...
                parents, edgeCosts, costs, neighborIdx, nodeCount);
            totalRewires = totalRewires + 1;
        end
    end
end

% Refresh bestCost from the goal node after all rewiring.
if ~isnan(goalIdx) && goalIdx <= nodeCount && isfinite(costs(goalIdx))
    bestCost = costs(goalIdx);
    info.bestCost = bestCost;
end
end

function isOnPath = isAncestor(candidateIdx, nodeIdx, parents)
%ISANCESTOR Check if candidateIdx is an ancestor of nodeIdx to prevent
%   rewiring cycles.
isOnPath = false;
current = parents(nodeIdx);
while current > 0
    if current == candidateIdx
        isOnPath = true;
        return;
    end
    current = parents(current);
end
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
