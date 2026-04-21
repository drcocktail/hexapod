function [path, pathZ, info] = planInformedRRTStar(startState, goalState, terrain, robot, options)
%PLANINFORMEDRRTSTAR Optimize paths with Informed RRT* in XY state space.

capacity = options.maxNodes + 2;
nodes = zeros(capacity, 2);
nodesZ = NaN(capacity, 1);
parents = zeros(capacity, 1);
costs = Inf(capacity, 1);
edgeCosts = zeros(capacity, 1);

info = baseInfo();
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

[spatialIndex, useSpatialIndex] = createIndexIfNeeded(terrain, options, startState);

for iteration = 1:options.maxNodes
    info.iterations = iteration;
    randomState = motionplanning.planning.sampleInformedState( ...
        startState, goalState, bestCost, terrain, options);

    nearestIdx = nearestTreeNode(nodes, nodeCount, randomState, spatialIndex, useSpatialIndex);
    newState = motionplanning.planning.steer(nodes(nearestIdx, :), randomState, options.stepSize);
    if norm(newState - nodes(nearestIdx, :)) < eps
        continue;
    end

    radius = motionplanning.planning.rrtStarRadius(options, nodeCount);
    nearIdx = nearbyTreeNodes(nodes, nodeCount, newState, radius, spatialIndex, useSpatialIndex);
    candidateParents = unique([nearestIdx; nearIdx(:)]);

    [bestParent, bestParentCost, bestParentEdgeCost, newZ, failReason] = chooseBestParent( ...
        candidateParents, newState, nodes, costs, bestCost, goalState, terrain, robot, options);
    if isnan(bestParent)
        info.failureReason = failReason;
        continue;
    end

    if nodeCount + 1 > capacity
        info.status = 'capacity_exhausted';
        break;
    end

    nodeCount = nodeCount + 1;
    nodes(nodeCount, :) = newState;
    nodesZ(nodeCount) = newZ;
    parents(nodeCount) = bestParent;
    costs(nodeCount) = bestParentCost;
    edgeCosts(nodeCount) = bestParentEdgeCost;
    if useSpatialIndex
        spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, nodeCount, newState);
    end

    [parents, costs, edgeCosts] = rewireNeighbors( ...
        nodeCount, nearIdx, nodes, parents, costs, edgeCosts, bestCost, terrain, robot, options);
    if ~isnan(goalIdx)
        bestCost = costs(goalIdx);
        info.bestCost = bestCost;
    end

    [goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, spatialIndex, info] = ...
        tryImproveGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
        spatialIndex, useSpatialIndex, goalState, goalZ, terrain, robot, options, info, iteration);
end

info.nodeCount = nodeCount;
info.bestCost = bestCost;
if isnan(goalIdx)
    path = [];
    pathZ = [];
    if strcmp(info.status, 'not_started')
        info.status = 'max_nodes_exhausted';
    end
    return;
end

[path, pathZ] = motionplanning.planning.reconstructPath(nodes, nodesZ, parents, goalIdx);
pathZ(end) = goalZ;
info.status = 'goal_reached';
end

function info = baseInfo()
info = struct( ...
    'status', 'not_started', ...
    'iterations', 0, ...
    'nodeCount', 0, ...
    'failureReason', '', ...
    'startValid', false, ...
    'goalValid', false, ...
    'bestCost', Inf);
end

function [path, pathZ, info] = invalidResult(info, status, reason)
path = [];
pathZ = [];
info.status = status;
info.failureReason = reason;
end

function [spatialIndex, useSpatialIndex] = createIndexIfNeeded(terrain, options, startState)
useSpatialIndex = isfield(options, 'nearestMode') && strcmpi(options.nearestMode, 'spatial_hash');
if useSpatialIndex
    spatialIndex = motionplanning.planning.createSpatialIndex(terrain, options.spatialCellSize);
    spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, 1, startState);
else
    spatialIndex = [];
end
end

function nearestIdx = nearestTreeNode(nodes, nodeCount, queryState, spatialIndex, useSpatialIndex)
if useSpatialIndex
    nearestIdx = motionplanning.planning.nearestNodeSpatial(spatialIndex, nodes, nodeCount, queryState);
else
    nearestIdx = motionplanning.planning.nearestNode(nodes, nodeCount, queryState);
end
end

function nearIdx = nearbyTreeNodes(nodes, nodeCount, queryState, radius, spatialIndex, useSpatialIndex)
if useSpatialIndex
    nearIdx = motionplanning.planning.nearNodesSpatial(spatialIndex, nodes, nodeCount, queryState, radius);
else
    nearIdx = motionplanning.planning.nearNodes(nodes, nodeCount, queryState, radius);
end
end

function [bestParent, bestCost, bestEdgeCost, newZ, failReason] = chooseBestParent( ...
    candidateParents, newState, nodes, costs, incumbentCost, goalState, terrain, robot, options)
bestParent = NaN;
bestCost = Inf;
bestEdgeCost = Inf;
newZ = NaN;
failReason = '';

for parentIdx = candidateParents(:)'
    lowerBound = costs(parentIdx) + ...
        motionplanning.planning.heuristicCost(nodes(parentIdx, :), newState) + ...
        motionplanning.planning.heuristicCost(newState, goalState);
    if lowerBound >= incumbentCost
        continue;
    end

    [candidateEdgeCost, candidateZ, edgeValid, edgeFailReason] = motionplanning.planning.edgeCost( ...
        nodes(parentIdx, :), newState, terrain, robot, options);
    if ~edgeValid
        failReason = edgeFailReason;
        continue;
    end

    candidateCost = costs(parentIdx) + candidateEdgeCost;
    if candidateCost < bestCost
        bestParent = parentIdx;
        bestCost = candidateCost;
        bestEdgeCost = candidateEdgeCost;
        newZ = candidateZ;
    end
end
end

function [parents, costs, edgeCosts] = rewireNeighbors( ...
    newIdx, nearIdx, nodes, parents, costs, edgeCosts, incumbentCost, terrain, robot, options)
for neighborIdx = nearIdx(:)'
    if neighborIdx == 1 || neighborIdx == newIdx || parents(newIdx) == neighborIdx || ...
            isAncestor(neighborIdx, newIdx, parents)
        continue;
    end

    lowerBound = costs(newIdx) + motionplanning.planning.heuristicCost(nodes(newIdx, :), nodes(neighborIdx, :));
    if lowerBound >= min(costs(neighborIdx), incumbentCost)
        continue;
    end

    [candidateEdgeCost, ~, edgeValid] = motionplanning.planning.edgeCost( ...
        nodes(newIdx, :), nodes(neighborIdx, :), terrain, robot, options);
    if ~edgeValid
        continue;
    end

    candidateCost = costs(newIdx) + candidateEdgeCost;
    if candidateCost < costs(neighborIdx)
        parents(neighborIdx) = newIdx;
        edgeCosts(neighborIdx) = candidateEdgeCost;
        costs(neighborIdx) = candidateCost;
        costs = motionplanning.planning.propagateCosts(parents, edgeCosts, costs, neighborIdx, numel(costs));
    end
end
end

function isOnPath = isAncestor(candidateIdx, nodeIdx, parents)
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

function [goalIdx, bestCost, nodeCount, nodes, nodesZ, parents, costs, edgeCosts, spatialIndex, info] = ...
    tryImproveGoal(nodeCount, goalIdx, bestCost, nodes, nodesZ, parents, costs, edgeCosts, ...
    spatialIndex, useSpatialIndex, goalState, goalZ, terrain, robot, options, info, iteration)

newIdx = nodeCount;
nearGoal = norm(nodes(newIdx, :) - goalState) <= max(options.goalTolerance, options.rewireRadius);
periodicCheck = mod(iteration, options.solutionCheckInterval) == 0;
if ~(nearGoal || periodicCheck)
    return;
end

lowerBound = costs(newIdx) + motionplanning.planning.heuristicCost(nodes(newIdx, :), goalState);
if lowerBound >= bestCost
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
    if useSpatialIndex
        spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, goalIdx, goalState);
    end
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
