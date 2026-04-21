function [joints, isValid] = solveLegIK(attachPoint, footPoint, robot)
%SOLVELEGIK Solve a simple coxa + femur/tibia IK chain in the leg plane.

joints = NaN(4, 3);
isValid = false;
if any(isnan(attachPoint)) || any(isnan(footPoint))
    return;
end

deltaXY = footPoint(1:2) - attachPoint(1:2);
xyDistance = norm(deltaXY);
if xyDistance < eps
    dirXY = [1, 0];
else
    dirXY = deltaXY / xyDistance;
end

coxaJoint = attachPoint + [dirXY * min(robot.L_coxa, xyDistance), 0];
target = footPoint - coxaJoint;
reach = norm(target);

minReach = robot.minFemurTibiaReach;
maxReach = robot.L_femur + robot.L_tibia;
isValid = reach >= minReach && reach <= maxReach && xyDistance >= robot.L_coxa * 0.5;

if reach < eps
    direction = [dirXY, 0];
    geometryReach = max(minReach + eps, eps);
else
    direction = target / reach;
    geometryReach = motionplanning.utils.clamp(reach, minReach + eps, maxReach - eps);
end

a = (robot.L_femur ^ 2 - robot.L_tibia ^ 2 + geometryReach ^ 2) / (2 * geometryReach);
hSq = max(robot.L_femur ^ 2 - a ^ 2, 0);

sideAxis = [-dirXY(2), dirXY(1), 0];
bendNormal = cross(sideAxis, direction);
normalLength = norm(bendNormal);
if normalLength < eps
    bendNormal = [0, 0, 1];
else
    bendNormal = bendNormal / normalLength;
    if bendNormal(3) < 0
        bendNormal = -bendNormal;
    end
end

kneeJoint = coxaJoint + a * direction + sqrt(hSq) * bendNormal;
joints = [attachPoint; coxaJoint; kneeJoint; footPoint];
end
