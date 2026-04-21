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
    'maxTerrainUnderBody', NaN, ...
    'legInfo', []);

bodyInside = x - robot.bodyRadius >= terrain.minX && ...
    x + robot.bodyRadius <= terrain.maxX && ...
    y - robot.bodyRadius >= terrain.minY && ...
    y + robot.bodyRadius <= terrain.maxY;
if ~bodyInside
    isValid = false;
    info.reason = 'body_outside_domain';
    return;
end

idxX = terrain.X(1, :) >= (x - robot.bodyRadius) & terrain.X(1, :) <= (x + robot.bodyRadius);
idxY = terrain.Y(:, 1) >= (y - robot.bodyRadius) & terrain.Y(:, 1) <= (y + robot.bodyRadius);
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
[legsReachable, legInfo] = motionplanning.robot.checkLegReachability([x, y, bodyZ], yaw, terrain, robot);

isValid = legsReachable;
info.bodyZ = bodyZ;
info.maxTerrainUnderBody = maxTerrainUnderBody;
info.legInfo = legInfo;
if isValid
    info.reason = 'valid';
else
    info.reason = 'leg_unreachable';
end
end
