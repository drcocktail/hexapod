function [pitch, roll] = estimateBodyOrientation(centerXY, yaw, terrain, robot)
%ESTIMATEBODYORIENTATION Estimate terrain-following chassis pitch and roll.

cw = cos(yaw);
sw = sin(yaw);
r  = robot.bodyRadius;

% Batch all four terrain probes into a single getTerrainHeight call.
% Row order: front, back, left, right.
px = centerXY(1) + r * [ cw; -cw; -sw;  sw];
py = centerXY(2) + r * [ sw; -sw;  cw; -cw];

z = motionplanning.environment.getTerrainHeight(px, py, terrain, 'nan');

if any(isnan(z))
    pitch = NaN;
    roll  = NaN;
    return;
end

pitch = atan2(z(1) - z(2), 2 * r) * 0.5;   % front vs back
roll  = atan2(z(4) - z(3), 2 * r) * 0.5;   % right vs left
end
