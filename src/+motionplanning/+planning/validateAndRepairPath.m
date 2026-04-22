function [path, pathZ, info] = validateAndRepairPath(gridPath, gridPathZ, terrain, robot, options)
%VALIDATEANDREPAIRPATH Full kinematic validation with local RRT-Connect repair.
%
%   Subjects each segment of the A* grid path to the rigorous isEdgeValid
%   checks (full body clearance, 6-leg IK, tripod stability).  If a
%   segment fails, deploys RRT-Connect locally to find a kinematic detour,
%   then splices the repair into the main path.
%
%   Key robustness features:
%     - Anchor expansion: if a repair endpoint fails isStateValid, the
%       region is expanded backward/forward to find a kinematically valid
%       anchor point.
%     - Scaled RRT budget: iteration count scales with repair distance.
%     - Handles consecutive failed segments as contiguous repair regions.
%
%   Returns a fully validated path or failure.

info = struct( ...
    'status', 'valid', ...
    'failureReason', '', ...
    'repairCount', 0, ...
    'segmentsTested', 0, ...
    'segmentsFailed', 0);

nWaypoints = size(gridPath, 1);
if nWaypoints < 2
    path = gridPath;
    pathZ = gridPathZ;
    return;
end

% --- Pre-validate all waypoints for isStateValid ---
% This tells us which waypoints can serve as valid RRT-Connect anchors.
initialYaw = atan2(gridPath(end, 2) - gridPath(1, 2), ...
                   gridPath(end, 1) - gridPath(1, 1));
waypointValid = false(nWaypoints, 1);
waypointZ     = gridPathZ;

for i = 1:nWaypoints
    [wv, wz] = motionplanning.planning.isStateValid( ...
        gridPath(i, :), terrain, robot, initialYaw);
    waypointValid(i) = wv;
    if wv
        waypointZ(i) = wz;
    end
end

% --- Walk each segment and validate ---
segValid    = false(nWaypoints - 1, 1);
segEndZ     = gridPathZ(2:end);
segFailReason = cell(nWaypoints - 1, 1);

for i = 1:(nWaypoints - 1)
    [edgeValid, zSamples, ~, failReason] = motionplanning.planning.isEdgeValid( ...
        gridPath(i, :), gridPath(i + 1, :), terrain, robot, options);

    segValid(i)      = edgeValid;
    segFailReason{i} = failReason;
    info.segmentsTested = info.segmentsTested + 1;

    if edgeValid
        segEndZ(i) = zSamples(end);
    end
end

failedIdx = find(~segValid);
info.segmentsFailed = numel(failedIdx);

if isempty(failedIdx)
    % All segments valid — return with updated Z values
    path  = gridPath;
    pathZ = waypointZ;
    for i = 1:(nWaypoints - 1)
        pathZ(i + 1) = segEndZ(i);
    end
    if waypointValid(1)
        pathZ(1) = waypointZ(1);
    end
    info.status = 'valid';
    fprintf('    [Validate] All %d segments passed kinematic checks.\n', ...
        info.segmentsTested);
    return;
end

fprintf('    [Validate] %d / %d segments failed. Attempting RRT-Connect repair...\n', ...
    info.segmentsFailed, info.segmentsTested);

% --- Group consecutive failures into contiguous repair regions ---
repairRegions = groupFailedSegments(failedIdx);

% --- Build per-region repairs ---
% Store: repairStartWP, repairEndWP, repairPath, repairZ
repairs = struct('startWP', {}, 'endWP', {}, 'path', {}, 'z', {});

baseMaxIter = getOpt(options, 'rrtConnectMaxIter', 500);
baseStepSize = getOpt(options, 'rrtConnectStepSize', 3.0);

for ri = 1:numel(repairRegions)
    region = repairRegions{ri};
    % region = [firstFailedSeg, lastFailedSeg]

    % --- Anchor expansion: find valid waypoints on each side ---
    % Walk backward from region start to find a kinematically valid anchor
    anchorStart = region(1);  % waypoint index of first failed segment's start
    while anchorStart > 1 && ~waypointValid(anchorStart)
        anchorStart = anchorStart - 1;
    end

    % Walk forward from region end to find a kinematically valid anchor
    anchorEnd = region(2) + 1;  % waypoint index after last failed segment
    while anchorEnd < nWaypoints && ~waypointValid(anchorEnd)
        anchorEnd = anchorEnd + 1;
    end

    startXY = gridPath(anchorStart, :);
    endXY   = gridPath(anchorEnd, :);
    startIsValid = waypointValid(anchorStart);
    endIsValid   = waypointValid(anchorEnd);

    fprintf('    [Repair %d/%d] Segments %d-%d, anchors WP%d(%.1f,%.1f)%s -> WP%d(%.1f,%.1f)%s\n', ...
        ri, numel(repairRegions), region(1), region(2), ...
        anchorStart, startXY(1), startXY(2), validTag(startIsValid), ...
        anchorEnd, endXY(1), endXY(2), validTag(endIsValid));

    if ~startIsValid || ~endIsValid
        % Cannot find valid anchors — last resort: skip this region
        % and try to connect through a wider sweep
        path = []; pathZ = [];
        info.status = 'repair_failed';
        info.failureReason = sprintf( ...
            'No valid anchor waypoints found for segments %d-%d (start=%s, goal=%s)', ...
            region(1), region(2), validTag(startIsValid), validTag(endIsValid));
        fprintf('    [Repair] FAILED: %s\n', info.failureReason);
        return;
    end

    % --- Scale RRT-Connect budget with repair distance ---
    % Use aggressive scaling: longer repairs in difficult terrain need
    % quadratically more iterations because most extensions are rejected.
    repairDist = norm(endXY - startXY);
    distRatio  = repairDist / baseStepSize;
    scaledIter = max(baseMaxIter, round(baseMaxIter * (1 + distRatio * 0.5 + distRatio^2 * 0.05)));

    % Build local repair options with scaled budget
    repairOptions = options;
    repairOptions.rrtConnectMaxIter = scaledIter;

    [repairPath, repairZ, repairInfo] = motionplanning.planning.planRRTConnect( ...
        startXY, endXY, terrain, robot, repairOptions);

    if isempty(repairPath)
        path = []; pathZ = [];
        info.status = 'repair_failed';
        info.failureReason = sprintf('RRT-Connect repair failed for segments %d-%d: %s (dist=%.1f, iter=%d)', ...
            region(1), region(2), repairInfo.failureReason, repairDist, scaledIter);
        fprintf('    [Repair] FAILED: %s\n', info.failureReason);
        return;
    end

    info.repairCount = info.repairCount + 1;
    fprintf('    [Repair] Success (%d nodes, %d/%d iterations, dist=%.1f)\n', ...
        size(repairPath, 1), repairInfo.iterations, scaledIter, repairDist);

    rep = struct();
    rep.startWP = anchorStart;
    rep.endWP   = anchorEnd;
    rep.path    = repairPath;
    rep.z       = repairZ;
    repairs(end + 1) = rep; %#ok<AGROW>
end

% --- Splice: walk waypoints, inserting repairs where needed ---
path  = [];
pathZ = [];
repairIdx = 1;  % index into repairs array
wp = 1;

while wp <= nWaypoints
    if repairIdx <= numel(repairs) && wp == repairs(repairIdx).startWP
        % Insert the repair path (includes anchor start and end)
        repairPath = repairs(repairIdx).path;
        repairZ    = repairs(repairIdx).z;

        if isempty(path)
            path  = repairPath;
            pathZ = repairZ;
        else
            % Avoid duplicating the anchor start (already in path)
            path  = [path;  repairPath(2:end, :)]; %#ok<AGROW>
            pathZ = [pathZ; repairZ(2:end)];       %#ok<AGROW>
        end

        % Skip all waypoints covered by this repair
        wp = repairs(repairIdx).endWP + 1;
        repairIdx = repairIdx + 1;
    else
        % Regular waypoint — add it
        if isempty(path)
            path  = gridPath(wp, :);
            pathZ = waypointZ(wp);
        else
            path  = [path;  gridPath(wp, :)]; %#ok<AGROW>
            pathZ = [pathZ; waypointZ(wp)];   %#ok<AGROW>
        end
        wp = wp + 1;
    end
end

% Ensure exact goal endpoint
path(end, :) = gridPath(end, :);

info.status = 'valid';
fprintf('    [Validate] Final path: %d waypoints (%d repairs applied)\n', ...
    size(path, 1), info.repairCount);
end

%% ---- Helpers ----

function tag = validTag(isValid)
if isValid
    tag = '[OK]';
else
    tag = '[INVALID]';
end
end

function regions = groupFailedSegments(failedIdx)
%GROUPFAILEDSEGMENTS Group consecutive failed segment indices into regions.
%   Each region is [firstFailed, lastFailed].

regions = {};
if isempty(failedIdx)
    return;
end

regionStart = failedIdx(1);
regionEnd   = failedIdx(1);

for k = 2:numel(failedIdx)
    if failedIdx(k) == regionEnd + 1
        regionEnd = failedIdx(k);
    else
        regions{end + 1} = [regionStart, regionEnd]; %#ok<AGROW>
        regionStart = failedIdx(k);
        regionEnd   = failedIdx(k);
    end
end
regions{end + 1} = [regionStart, regionEnd];
end

function value = getOpt(options, name, default)
if isfield(options, name)
    value = options.(name);
else
    value = default;
end
end
