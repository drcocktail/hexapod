function [feet, inside] = footTargets(centerXY, yaw, terrain, robot, targetMode)
%FOOTTARGETS Compute nominal foot placements and terrain contact heights.

if nargin < 5
    targetMode = 'planning';
end

if strcmpi(targetMode, 'visual')
    targetRadius = robot.visualFootRadius;
else
    targetRadius = robot.nominalFootRadius;
end

feet = zeros(6, 3);
inside = true(6, 1);
for leg = 1:6
    heading = yaw + robot.baseAngles(leg);
    footX = centerXY(1) + targetRadius * cos(heading);
    footY = centerXY(2) + targetRadius * sin(heading);
    footZ = motionplanning.environment.getTerrainHeight(footX, footY, terrain, 'nan');

    feet(leg, :) = [footX, footY, footZ];
    inside(leg) = ~isnan(footZ);
end
end
