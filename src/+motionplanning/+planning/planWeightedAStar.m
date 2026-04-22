function [path, pathZ, info] = planWeightedAStar(startState, goalState, terrain, robot, options)
%PLANWEIGHTEDASTAR Deterministic grid-based Weighted A* planner.
%
%   Operates on the precomputed traversability map from
%   buildTraversabilityMap.  Searches the 8-connected terrain grid using
%   an inflated heuristic (epsilon * Euclidean) for bounded-suboptimal
%   paths.  The grid path is then validated and repaired via
%   validateAndRepairPath before being returned.
%
%   Guarantees:
%     - Resolution-complete on the discrete grid
%     - Path cost <= epsilon * optimal grid path cost
%     - Deterministic: same terrain + endpoints -> same path every time

info = struct( ...
    'status', 'not_started', ...
    'iterations', 0, ...
    'nodeCount', 0, ...
    'failureReason', '', ...
    'startValid', false, ...
    'goalValid', false, ...
    'bestCost', Inf, ...
    'gridSearchTime', 0, ...
    'validationTime', 0, ...
    'repairCount', 0);

epsilon = getOpt(options, 'astarEpsilon', 3.0);

% --- 1. Build traversability map ---
fprintf('  [A*] Building traversability map...\n');
tic;
[traversable, costMap] = motionplanning.planning.buildTraversabilityMap( ...
    terrain, robot, options);
info.gridSearchTime = toc;
fprintf('  [A*] Traversability map built in %.3f s. Traversable: %d / %d cells (%.1f%%)\n', ...
    info.gridSearchTime, sum(traversable(:)), numel(traversable), ...
    100 * sum(traversable(:)) / numel(traversable));

% --- 2. Convert start/goal to grid indices ---
gs = terrain.gridSpacing;
[nRows, nCols] = size(terrain.Z);

startCol = clampIdx(round((startState(1) - terrain.minX) / gs) + 1, 1, nCols);
startRow = clampIdx(round((startState(2) - terrain.minY) / gs) + 1, 1, nRows);
goalCol  = clampIdx(round((goalState(1) - terrain.minX) / gs) + 1, 1, nCols);
goalRow  = clampIdx(round((goalState(2) - terrain.minY) / gs) + 1, 1, nRows);

info.startValid = traversable(startRow, startCol);
info.goalValid  = traversable(goalRow, goalCol);

if ~info.startValid
    path = []; pathZ = [];
    info.status = 'invalid_start';
    info.failureReason = 'start_cell_not_traversable';
    fprintf('  [A*] Start cell (%d,%d) is not traversable.\n', startRow, startCol);
    return;
end
if ~info.goalValid
    path = []; pathZ = [];
    info.status = 'invalid_goal';
    info.failureReason = 'goal_cell_not_traversable';
    fprintf('  [A*] Goal cell (%d,%d) is not traversable.\n', goalRow, goalCol);
    return;
end

% --- 3. Weighted A* search on 8-connected grid ---
fprintf('  [A*] Starting grid search (epsilon=%.1f)...\n', epsilon);
searchTic = tic;

totalCells = nRows * nCols;
gCost   = Inf(nRows, nCols);
parentR = zeros(nRows, nCols);
parentC = zeros(nRows, nCols);
closed  = false(nRows, nCols);

gCost(startRow, startCol) = 0;

% 8-connected neighbor offsets: [dRow, dCol, moveCost]
dr = [-1, -1, -1,  0, 0,  1, 1, 1];
dc = [-1,  0,  1, -1, 1, -1, 0, 1];
moveDist = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)] * gs;

% Binary min-heap: each entry is [fCost, linearIndex]
heapCap  = min(totalCells, 200000);
heap     = zeros(heapCap, 2);
heapSize = 0;

goalLinear = sub2lin(goalRow, goalCol, nRows);
h0 = epsilon * gs * sqrt((goalRow - startRow)^2 + (goalCol - startCol)^2);
[heap, heapSize] = heapPush(heap, heapSize, h0, sub2lin(startRow, startCol, nRows));

iterations = 0;
found = false;

while heapSize > 0
    [heap, heapSize, fVal, linIdx] = heapPop(heap, heapSize); %#ok<ASGLU>
    [cr, cc] = lin2sub(linIdx, nRows);

    if closed(cr, cc)
        continue;
    end
    closed(cr, cc) = true;
    iterations = iterations + 1;

    if cr == goalRow && cc == goalCol
        found = true;
        break;
    end

    currentG = gCost(cr, cc);

    for k = 1:8
        nr = cr + dr(k);
        nc = cc + dc(k);

        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue;
        end
        if closed(nr, nc) || ~traversable(nr, nc)
            continue;
        end

        % Edge cost: movement distance * local terrain cost
        edgeCost = moveDist(k) * 0.5 * (costMap(cr, cc) + costMap(nr, nc));
        tentativeG = currentG + edgeCost;

        if tentativeG < gCost(nr, nc)
            gCost(nr, nc) = tentativeG;
            parentR(nr, nc) = cr;
            parentC(nr, nc) = cc;

            h = epsilon * gs * sqrt((goalRow - nr)^2 + (goalCol - nc)^2);
            fCost = tentativeG + h;
            [heap, heapSize] = heapPush(heap, heapSize, fCost, sub2lin(nr, nc, nRows));
        end
    end
end

searchTime = toc(searchTic);
info.gridSearchTime = info.gridSearchTime + searchTime;
info.iterations = iterations;
fprintf('  [A*] Grid search completed in %.3f s. Iterations: %d. Found: %s\n', ...
    searchTime, iterations, string(found));

if ~found
    path = []; pathZ = [];
    info.status = 'no_path_on_grid';
    info.failureReason = 'grid_search_exhausted';
    return;
end

% --- 4. Reconstruct grid path ---
gridPath = reconstructGridPath(parentR, parentC, startRow, startCol, goalRow, goalCol, nRows);
nWaypoints = size(gridPath, 1);

% Convert grid indices to XY world coordinates
pathXY = zeros(nWaypoints, 2);
pathZ  = zeros(nWaypoints, 1);
for k = 1:nWaypoints
    r = gridPath(k, 1);
    c = gridPath(k, 2);
    pathXY(k, :) = [terrain.xVec(c), terrain.yVec(r)];
    pathZ(k) = terrain.Z(r, c) + robot.clearance + robot.bodyHeight / 2;
end

% Force exact start/goal coordinates
pathXY(1, :)   = startState;
pathXY(end, :) = goalState;

info.bestCost = gCost(goalRow, goalCol);
info.nodeCount = iterations;

fprintf('  [A*] Raw grid path: %d waypoints, cost %.2f\n', nWaypoints, info.bestCost);

% --- 5. Simplify: remove collinear waypoints to reduce validation load ---
pathXY = simplifyPath(pathXY, pathZ, gs);
pathZ  = zeros(size(pathXY, 1), 1);
for k = 1:size(pathXY, 1)
    tz = motionplanning.environment.getTerrainHeight( ...
        pathXY(k, 1), pathXY(k, 2), terrain, 'clamp');
    pathZ(k) = tz + robot.clearance + robot.bodyHeight / 2;
end
fprintf('  [A*] Simplified to %d waypoints.\n', size(pathXY, 1));

% --- 6. Validate and repair with full kinematic checks ---
fprintf('  [A*] Validating path with full kinematic checks...\n');
validTic = tic;
[path, pathZ, repairInfo] = motionplanning.planning.validateAndRepairPath( ...
    pathXY, pathZ, terrain, robot, options);
info.validationTime = toc(validTic);
info.repairCount = repairInfo.repairCount;

if isempty(path)
    info.status = repairInfo.status;
    info.failureReason = repairInfo.failureReason;
    fprintf('  [A*] Path validation FAILED: %s\n', info.failureReason);
else
    % --- 7. Smooth and densify the validated path ---
    fprintf('  [A*] Smoothing and densifying path...\n');
    smoothTic = tic;
    [path, pathZ] = motionplanning.planning.smoothPath( ...
        path, pathZ, terrain, robot, options);
    smoothTime = toc(smoothTic);

    info.status = 'goal_reached';
    info.bestCost = gCost(goalRow, goalCol);
    fprintf('  [A*] Complete. %d repairs. %d final waypoints. Time: %.3f s (search) + %.3f s (validation) + %.3f s (smooth)\n', ...
        info.repairCount, size(path, 1), info.gridSearchTime, info.validationTime, smoothTime);
end

end

%% ---- Helper functions ----

function idx = clampIdx(idx, lo, hi)
idx = max(lo, min(hi, idx));
end

function lin = sub2lin(r, c, nRows)
lin = (c - 1) * nRows + r;
end

function [r, c] = lin2sub(lin, nRows)
c = floor((lin - 1) / nRows) + 1;
r = lin - (c - 1) * nRows;
end

function gridPath = reconstructGridPath(parentR, parentC, sr, sc, gr, gc, nRows) %#ok<INUSD>
% Count path length first
count = 0;
cr = gr; cc = gc;
while cr ~= 0
    count = count + 1;
    pr = parentR(cr, cc);
    pc = parentC(cr, cc);
    cr = pr; cc = pc;
end

gridPath = zeros(count, 2);
cr = gr; cc = gc;
for k = count:-1:1
    gridPath(k, :) = [cr, cc];
    pr = parentR(cr, cc);
    pc = parentC(cr, cc);
    cr = pr; cc = pc;
end
end

function simplified = simplifyPath(pathXY, pathZ, tolerance) %#ok<INUSD>
%SIMPLIFYPATH Remove collinear waypoints, then enforce max segment length.
n = size(pathXY, 1);
if n <= 2
    simplified = pathXY;
    return;
end

% Maximum segment length: prevents mega-segments that span difficult
% terrain and overwhelm the RRT-Connect repair budget.
maxSegLen = tolerance * 15;  % ~15 grid cells ≈ stepSize * 3

keep = true(n, 1);
keep(1)   = true;
keep(end) = true;

i = 1;
while i < n
    % Find the farthest point we can skip to while staying nearly collinear
    j = i + 2;
    while j <= n
        segDir = pathXY(j, :) - pathXY(i, :);
        segLen = norm(segDir);
        if segLen < eps
            break;
        end

        % Also break if segment would exceed max length
        if segLen > maxSegLen
            break;
        end

        segDir = segDir / segLen;

        allClose = true;
        for k = (i+1):(j-1)
            v = pathXY(k, :) - pathXY(i, :);
            proj = dot(v, segDir);
            perpDist = norm(v - proj * segDir);
            if perpDist > tolerance * 1.5
                allClose = false;
                break;
            end
        end

        if ~allClose
            break;
        end
        j = j + 1;
    end

    % Mark intermediate points for removal
    for k = (i+1):(j-2)
        keep(k) = false;
    end
    i = j - 1;
end

simplified = pathXY(keep, :);
end

function value = getOpt(options, name, default)
if isfield(options, name)
    value = options.(name);
else
    value = default;
end
end

%% ---- Binary Min-Heap ----

function [heap, heapSize] = heapPush(heap, heapSize, fCost, linIdx)
heapSize = heapSize + 1;
if heapSize > size(heap, 1)
    heap = [heap; zeros(size(heap, 1), 2)];  % double capacity
end
heap(heapSize, :) = [fCost, linIdx];

% Sift up
idx = heapSize;
while idx > 1
    parent = floor(idx / 2);
    if heap(parent, 1) > heap(idx, 1)
        heap([parent, idx], :) = heap([idx, parent], :);
        idx = parent;
    else
        break;
    end
end
end

function [heap, heapSize, fCost, linIdx] = heapPop(heap, heapSize)
fCost  = heap(1, 1);
linIdx = heap(1, 2);

heap(1, :) = heap(heapSize, :);
heapSize = heapSize - 1;

% Sift down
idx = 1;
while true
    left  = 2 * idx;
    right = 2 * idx + 1;
    smallest = idx;

    if left <= heapSize && heap(left, 1) < heap(smallest, 1)
        smallest = left;
    end
    if right <= heapSize && heap(right, 1) < heap(smallest, 1)
        smallest = right;
    end

    if smallest ~= idx
        heap([idx, smallest], :) = heap([smallest, idx], :);
        idx = smallest;
    else
        break;
    end
end
end
