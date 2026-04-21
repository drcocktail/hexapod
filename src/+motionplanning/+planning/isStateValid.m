function [isValid, bodyZ, info] = isStateValid(state, terrain, robot, yaw)
%ISSTATEVALID Validate body clearance and nominal six-leg reachability.

if nargin < 4
    yaw = 0;
end

x = state(1);
y = state(2);
bodyZ = NaN;
info = struct( ...
    'reason', '', ...
    'bodyZ', NaN, ...
    'pitch', NaN, ...
    'roll', NaN, ...
    'maxTerrainUnderBody', NaN, ...
    'legInfo', [], ...
    'stabilityInfo', []);

bodyInside = x - robot.bodyRadius >= terrain.minX && ...
    x + robot.bodyRadius <= terrain.maxX && ...
    y - robot.bodyRadius >= terrain.minY && ...
    y + robot.bodyRadius <= terrain.maxY;
if ~bodyInside
    isValid = false;
    info.reason = 'body_outside_domain';
    return;
end

[idxX, idxY] = motionplanning.environment.localGridWindow(x, y, robot.bodyRadius, terrain);
localX = terrain.X(idxY, idxX);
localY = terrain.Y(idxY, idxX);
localZ = terrain.Z(idxY, idxX);

distSq = (localX - x) .^ 2 + (localY - y) .^ 2;
mask = distSq <= robot.bodyRadius ^ 2;
if any(mask(:))
    maxTerrainUnderBody = max(localZ(mask));
else
    maxTerrainUnderBody = motionplanning.environment.getTerrainHeight(x, y, terrain, 'nan');
end

if isnan(maxTerrainUnderBody)
    isValid = false;
    info.reason = 'terrain_query_outside_domain';
    return;
end

bodyZ = maxTerrainUnderBody + robot.clearance + robot.bodyHeight / 2;
[pitch, roll] = motionplanning.robot.estimateBodyOrientation([x, y], yaw, terrain, robot);
if isnan(pitch) || isnan(roll)
    isValid = false;
    info.reason = 'orientation_query_outside_domain';
    return;
end
if abs(pitch) > robot.maxPitch || abs(roll) > robot.maxRoll
    isValid = false;
    info.reason = 'body_orientation_limit';
    info.bodyZ = bodyZ;
    info.pitch = pitch;
    info.roll = roll;
    return;
end

[legsReachable, legInfo] = motionplanning.robot.checkLegReachability( ...
    [x, y, bodyZ], yaw, pitch, roll, terrain, robot);

info.bodyZ = bodyZ;
info.pitch = pitch;
info.roll = roll;
info.maxTerrainUnderBody = maxTerrainUnderBody;
info.legInfo = legInfo;
if ~legsReachable
    isValid = false;
    info.reason = 'leg_unreachable';
    return;
end

[stable, stabilityInfo] = motionplanning.robot.checkStaticStability( ...
    legInfo.feet, [x, y], robot.stabilityMargin);
info.stabilityInfo = stabilityInfo;
if ~stable
    isValid = false;
    info.reason = 'unstable_tripod_support';
    return;
end

isValid = true;
info.reason = 'valid';
end
