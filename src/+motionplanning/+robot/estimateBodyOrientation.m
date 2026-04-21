function [pitch, roll] = estimateBodyOrientation(centerXY, yaw, terrain, robot)
%ESTIMATEBODYORIENTATION Estimate terrain-following chassis pitch and roll.

forward = [cos(yaw), sin(yaw)];
left = [cos(yaw + pi / 2), sin(yaw + pi / 2)];

frontXY = centerXY + robot.bodyRadius * forward;
backXY = centerXY - robot.bodyRadius * forward;
leftXY = centerXY + robot.bodyRadius * left;
rightXY = centerXY - robot.bodyRadius * left;

zFwd = motionplanning.environment.getTerrainHeight(frontXY(1), frontXY(2), terrain, 'nan');
zBck = motionplanning.environment.getTerrainHeight(backXY(1), backXY(2), terrain, 'nan');
zLft = motionplanning.environment.getTerrainHeight(leftXY(1), leftXY(2), terrain, 'nan');
zRgt = motionplanning.environment.getTerrainHeight(rightXY(1), rightXY(2), terrain, 'nan');

if any(isnan([zFwd, zBck, zLft, zRgt]))
    pitch = NaN;
    roll = NaN;
    return;
end

pitch = atan2(zFwd - zBck, 2 * robot.bodyRadius) * 0.5;
roll = atan2(zRgt - zLft, 2 * robot.bodyRadius) * 0.5;
end
