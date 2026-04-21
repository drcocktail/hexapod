function [path, pathZ, info] = planRRT(startState, goalState, terrain, robot, options)
%PLANRRT Build an RRT under volumetric body and leg reachability checks.

capacity = options.maxNodes + 2;
nodes = zeros(capacity, 2);
nodesZ = NaN(capacity, 1);
parents = zeros(capacity, 1);

info = struct( ...
    'status', 'not_started', ...
    'iterations', 0, ...
    'nodeCount', 0, ...
    'failureReason', '', ...
    'startValid', false, ...
    'goalValid', false);

initialYaw = atan2(goalState(2) - startState(2), goalState(1) - startState(1));
[startValid, startZ, startInfo] = motionplanning.planning.isStateValid(startState, terrain, robot, initialYaw);
[goalValid, goalZ, goalInfo] = motionplanning.planning.isStateValid(goalState, terrain, robot, initialYaw);

info.startValid = startValid;
info.goalValid = goalValid;
if ~startValid
    path = [];
    pathZ = [];
    info.status = 'invalid_start';
    info.failureReason = startInfo.reason;
    return;
end
if ~goalValid
    path = [];
    pathZ = [];
    info.status = 'invalid_goal';
    info.failureReason = goalInfo.reason;
    return;
end

nodes(1, :) = startState;
nodesZ(1) = startZ;
nodeCount = 1;
goalIdx = NaN;
useSpatialIndex = isfield(options, 'nearestMode') && strcmpi(options.nearestMode, 'spatial_hash');
if useSpatialIndex
    if isfield(options, 'spatialCellSize')
        spatialCellSize = options.spatialCellSize;
    else
        spatialCellSize = options.stepSize * 4;
    end
    spatialIndex = motionplanning.planning.createSpatialIndex(terrain, spatialCellSize);
    spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, 1, startState);
else
    spatialIndex = [];
end

maxIterations = options.maxNodes * 20;
for iteration = 1:maxIterations
    if nodeCount >= options.maxNodes
        break;
    end
    info.iterations = iteration;
    randomState = motionplanning.planning.sampleState(goalState, terrain, options);
    if useSpatialIndex
        nearestIdx = motionplanning.planning.nearestNodeSpatial( ...
            spatialIndex, nodes, nodeCount, randomState);
    else
        nearestIdx = motionplanning.planning.nearestNode(nodes, nodeCount, randomState);
    end
    newState = motionplanning.planning.steer(nodes(nearestIdx, :), randomState, options.stepSize);

    if norm(newState - nodes(nearestIdx, :)) < eps
        continue;
    end

    [edgeValid, zSamples, ~, failReason] = motionplanning.planning.isEdgeValid( ...
        nodes(nearestIdx, :), newState, terrain, robot, options);
    if ~edgeValid
        info.failureReason = failReason;
        continue;
    end

    nodeCount = nodeCount + 1;
    if nodeCount > capacity
        info.status = 'capacity_exhausted';
        break;
    end

    nodes(nodeCount, :) = newState;
    nodesZ(nodeCount) = zSamples(end);
    parents(nodeCount) = nearestIdx;
    if useSpatialIndex
        spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, nodeCount, newState);
    end

    if norm(newState - goalState) <= options.goalTolerance
        [goalEdgeValid, goalZSamples, ~, failReason] = motionplanning.planning.isEdgeValid( ...
            newState, goalState, terrain, robot, options);
        if goalEdgeValid
            nodeCount = nodeCount + 1;
            nodes(nodeCount, :) = goalState;
            nodesZ(nodeCount) = goalZSamples(end);
            parents(nodeCount) = nodeCount - 1;
            if useSpatialIndex
                spatialIndex = motionplanning.planning.insertSpatialNode(spatialIndex, nodeCount, goalState);
            end
            goalIdx = nodeCount;
            info.status = 'goal_reached';
            break;
        end
        info.failureReason = failReason;
    end
end

info.nodeCount = nodeCount;
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
end
