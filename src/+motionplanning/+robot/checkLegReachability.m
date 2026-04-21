function [isReachable, legInfo] = checkLegReachability(bodyPose, yaw, pitch, roll, terrain, robot)
%CHECKLEGREACHABILITY Validate all six nominal footholds for a body pose.

R = motionplanning.utils.rotationMatrix(yaw, pitch, roll);
[feet, feetInside] = motionplanning.robot.footTargets(bodyPose(1:2), yaw, terrain, robot, 'planning');

valid = false(6, 1);
attachments = zeros(6, 3);
jointChains = NaN(4, 3, 6);

for leg = 1:6
    localAttachment = [ ...
        robot.bodyRadius * cos(robot.baseAngles(leg)), ...
        robot.bodyRadius * sin(robot.baseAngles(leg)), ...
        0];
    attachments(leg, :) = bodyPose + (R * localAttachment')';

    [jointChains(:, :, leg), legIKValid] = motionplanning.robot.solveLegIK( ...
        attachments(leg, :), feet(leg, :), robot);

    verticalDrop = attachments(leg, 3) - feet(leg, 3);
    verticalValid = verticalDrop <= robot.maxReach && verticalDrop >= -robot.maxStepZ;
    valid(leg) = feetInside(leg) && legIKValid && verticalValid;
end

isReachable = all(valid);
legInfo = struct( ...
    'valid', valid, ...
    'feet', feet, ...
    'feetInside', feetInside, ...
    'attachments', attachments, ...
    'jointChains', jointChains);
end
