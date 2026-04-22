function [smoothed, smoothedZ] = smoothPath(pathXY, pathZ, terrain, robot, options)
%SMOOTHPATH Refine a validated path via shortcutting and densification.
%
%   Three-stage pipeline:
%     1. GREEDY SHORTCUTTING — walk the path and skip intermediate waypoints
%        whenever a direct connection is kinematically valid.  This removes
%        the grid-aligned staircase pattern from A* and the random zig-zags
%        from RRT-Connect repairs.
%
%     2. RANDOM SHORTCUTTING — pick random pairs of points on the path and
%        try to connect them directly.  This is the PRM-style optimization
%        pass that finds shorter, smoother corridors.
%
%     3. DENSIFICATION — resample the smoothed path at uniform spacing so
%        the robot follows a high-resolution trajectory instead of gliding
%        over long segments.

stepSize  = getOpt(options, 'stepSize', 5.0);
edgeRes   = getOpt(options, 'edgeCheckResolution', 2.0);
targetSpacing = stepSize * 0.6;  % densification target: ~3 units between waypoints
randomShortcutAttempts = getOpt(options, 'smoothShortcutAttempts', 150);

n = size(pathXY, 1);
if n <= 2
    [smoothed, smoothedZ] = densifyPath(pathXY, pathZ, targetSpacing, terrain, robot);
    return;
end

fprintf('    [Smooth] Input: %d waypoints, length=%.1f\n', n, pathLength(pathXY));

% --- Stage 1: Greedy sequential shortcutting ---
% Walk forward through the path.  From each anchor point, try to skip
% as far ahead as possible while maintaining a valid edge.
greedyXY = pathXY(1, :);
greedyZ  = pathZ(1);
anchor = 1;

while anchor < n
    % Try to skip as far ahead as possible
    bestTarget = anchor + 1;

    % Test increasingly distant targets
    for target = min(anchor + 2, n) : n
        [edgeValid, zSamples, ~, ~] = motionplanning.planning.isEdgeValid( ...
            pathXY(anchor, :), pathXY(target, :), terrain, robot, options);
        if edgeValid
            bestTarget = target;
        else
            % Once we fail, don't try further — the path likely curves away
            break;
        end
    end

    greedyXY = [greedyXY; pathXY(bestTarget, :)]; %#ok<AGROW>
    greedyZ  = [greedyZ;  pathZ(bestTarget)];     %#ok<AGROW>
    anchor = bestTarget;
end

fprintf('    [Smooth] Greedy shortcut: %d -> %d waypoints, length=%.1f\n', ...
    n, size(greedyXY, 1), pathLength(greedyXY));

% --- Stage 2: Random shortcutting (PRM-style optimization) ---
% Pick two random points on the path and try to connect them directly.
% If valid and shorter, replace the intermediate segment.
currentXY = greedyXY;
currentZ  = greedyZ;

improvements = 0;
for attempt = 1:randomShortcutAttempts
    nCur = size(currentXY, 1);
    if nCur <= 2
        break;
    end

    % Pick two random waypoint indices with at least one waypoint between them
    idx1 = randi(nCur - 2);
    idx2 = idx1 + randi(nCur - idx1 - 1) + 1;
    if idx2 > nCur
        continue;
    end

    % Would the shortcut actually be shorter?
    segmentLength = sumSegmentLength(currentXY, idx1, idx2);
    directLength  = norm(currentXY(idx2, :) - currentXY(idx1, :));
    if directLength >= segmentLength * 0.95
        continue;  % not enough improvement to justify the edge check
    end

    % Validate the direct connection
    [edgeValid, zSamples, ~, ~] = motionplanning.planning.isEdgeValid( ...
        currentXY(idx1, :), currentXY(idx2, :), terrain, robot, options);
    if ~edgeValid
        continue;
    end

    % Replace the intermediate waypoints with the direct connection
    currentXY = [currentXY(1:idx1, :); currentXY(idx2:end, :)];
    currentZ  = [currentZ(1:idx1);     currentZ(idx2:end)];
    % Update Z at the connection point
    currentZ(idx1 + 1) = zSamples(end);
    improvements = improvements + 1;
end

fprintf('    [Smooth] Random shortcut: %d improvements in %d attempts, %d waypoints, length=%.1f\n', ...
    improvements, randomShortcutAttempts, size(currentXY, 1), pathLength(currentXY));

% --- Stage 3: Densification ---
% Resample at uniform spacing for smooth robot traversal.
[smoothed, smoothedZ] = densifyPath(currentXY, currentZ, targetSpacing, terrain, robot);

fprintf('    [Smooth] Densified to %d waypoints (spacing=%.1f), final length=%.1f\n', ...
    size(smoothed, 1), targetSpacing, pathLength(smoothed));
end

%% ---- Helpers ----

function [denseXY, denseZ] = densifyPath(pathXY, pathZ, targetSpacing, terrain, robot)
%DENSIFYPATH Resample a polyline at approximately uniform spacing.

nOrig = size(pathXY, 1);
if nOrig <= 1
    denseXY = pathXY;
    denseZ  = pathZ;
    return;
end

% Compute cumulative arc length
arcLen = zeros(nOrig, 1);
for i = 2:nOrig
    arcLen(i) = arcLen(i-1) + norm(pathXY(i,:) - pathXY(i-1,:));
end
totalLen = arcLen(end);

if totalLen < targetSpacing
    denseXY = pathXY;
    denseZ  = pathZ;
    return;
end

% Number of output points
nOut = max(2, ceil(totalLen / targetSpacing) + 1);
targetArc = linspace(0, totalLen, nOut)';

denseXY = zeros(nOut, 2);
denseZ  = zeros(nOut, 1);
denseXY(1, :) = pathXY(1, :);
denseZ(1)     = pathZ(1);
denseXY(end, :) = pathXY(end, :);
denseZ(end)     = pathZ(end);

seg = 1;
for k = 2:(nOut - 1)
    t = targetArc(k);

    % Advance to the correct segment
    while seg < nOrig - 1 && arcLen(seg + 1) < t
        seg = seg + 1;
    end

    % Interpolate within segment
    segStart = arcLen(seg);
    segEnd   = arcLen(seg + 1);
    segLen   = segEnd - segStart;
    if segLen < eps
        alpha = 0;
    else
        alpha = (t - segStart) / segLen;
    end

    denseXY(k, :) = pathXY(seg, :) + alpha * (pathXY(seg+1, :) - pathXY(seg, :));

    % Query terrain for accurate Z
    tz = motionplanning.environment.getTerrainHeight( ...
        denseXY(k, 1), denseXY(k, 2), terrain, 'clamp');
    denseZ(k) = tz + robot.clearance + robot.bodyHeight / 2;
end

% Also fix start/end Z from terrain
tz1 = motionplanning.environment.getTerrainHeight(denseXY(1,1), denseXY(1,2), terrain, 'clamp');
denseZ(1) = tz1 + robot.clearance + robot.bodyHeight / 2;
tzN = motionplanning.environment.getTerrainHeight(denseXY(end,1), denseXY(end,2), terrain, 'clamp');
denseZ(end) = tzN + robot.clearance + robot.bodyHeight / 2;
end

function len = pathLength(pathXY)
%PATHLENGTH Total Euclidean length of a polyline.
diffs = diff(pathXY, 1, 1);
len = sum(sqrt(sum(diffs .^ 2, 2)));
end

function len = sumSegmentLength(pathXY, idx1, idx2)
%SUMSEGMENTLENGTH Sum of edge lengths between indices idx1 and idx2.
len = 0;
for k = idx1:(idx2-1)
    len = len + norm(pathXY(k+1, :) - pathXY(k, :));
end
end

function value = getOpt(options, name, default)
if isfield(options, name)
    value = options.(name);
else
    value = default;
end
end
