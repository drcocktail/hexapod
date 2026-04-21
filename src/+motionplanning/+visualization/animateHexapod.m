function animateHexapod(ax, pathXY, pathZ, terrain, robot, options)
%ANIMATEHEXAPOD Render the robot moving along the planned path.

[chassisVertices, chassisFaces] = motionplanning.robot.chassisPrototype(robot);
hChassis = patch(ax, 'Faces', chassisFaces, 'Vertices', chassisVertices, ...
    'FaceColor', [0.15, 0.25, 0.35], ...
    'EdgeColor', [0.0, 0.8, 0.8], ...
    'LineWidth', 1.5);

hLegs = gobjects(1, 6);
hJoints = gobjects(1, 6);
for leg = 1:6
    hLegs(leg) = plot3(ax, zeros(1, 4), zeros(1, 4), zeros(1, 4), ...
        'Color', [0.7, 0.7, 0.7], 'LineWidth', 4);
    hJoints(leg) = plot3(ax, zeros(1, 4), zeros(1, 4), zeros(1, 4), ...
        'o', 'MarkerSize', 6, 'MarkerFaceColor', [1, 0.4, 0], 'MarkerEdgeColor', 'k');
end

numWaypoints = size(pathXY, 1);
for waypoint = 1:(numWaypoints - 1)
    startXY = pathXY(waypoint, :);
    endXY = pathXY(waypoint + 1, :);
    startZ = pathZ(waypoint);
    endZ = pathZ(waypoint + 1);
    theta = atan2(endXY(2) - startXY(2), endXY(1) - startXY(1));

    for frame = 1:options.framesPerWaypoint
        alpha = frame / options.framesPerWaypoint;
        ease = (1 - cos(alpha * pi)) / 2;

        currXY = startXY + ease * (endXY - startXY);
        currZ = startZ + ease * (endZ - startZ);

        zFwd = motionplanning.environment.getTerrainHeight( ...
            currXY(1) + robot.bodyRadius * cos(theta), ...
            currXY(2) + robot.bodyRadius * sin(theta), terrain, 'clamp');
        zBck = motionplanning.environment.getTerrainHeight( ...
            currXY(1) - robot.bodyRadius * cos(theta), ...
            currXY(2) - robot.bodyRadius * sin(theta), terrain, 'clamp');
        zLft = motionplanning.environment.getTerrainHeight( ...
            currXY(1) + robot.bodyRadius * cos(theta + pi / 2), ...
            currXY(2) + robot.bodyRadius * sin(theta + pi / 2), terrain, 'clamp');
        zRgt = motionplanning.environment.getTerrainHeight( ...
            currXY(1) + robot.bodyRadius * cos(theta - pi / 2), ...
            currXY(2) + robot.bodyRadius * sin(theta - pi / 2), terrain, 'clamp');

        pitch = atan2(zFwd - zBck, 2 * robot.bodyRadius) * 0.5;
        roll = atan2(zRgt - zLft, 2 * robot.bodyRadius) * 0.5;
        R = motionplanning.utils.rotationMatrix(theta, pitch, roll);

        transformedVertices = (R * chassisVertices')' + repmat([currXY, currZ], 12, 1);
        set(hChassis, 'Vertices', transformedVertices);

        updateLegs(hLegs, hJoints, transformedVertices, currXY, currZ, theta, alpha, terrain, robot);

        drawnow limitrate;
        pause(options.frameDelay);
    end
end
end

function updateLegs(hLegs, hJoints, chassisVertices, currXY, currZ, theta, alpha, terrain, robot)
for leg = 1:6
    attachPoint = chassisVertices(leg, :);
    legHeading = theta + robot.baseAngles(leg);
    footX = currXY(1) + robot.visualFootRadius * cos(legHeading);
    footY = currXY(2) + robot.visualFootRadius * sin(legHeading);
    footZ = motionplanning.environment.getTerrainHeight(footX, footY, terrain, 'clamp');

    isLifting = mod(leg + round(alpha * 2), 2) == 0;
    if isLifting
        footZ = footZ + 1.5 * sin(alpha * pi);
    end

    [joints, isValid] = motionplanning.robot.solveLegIK( ...
        attachPoint, [footX, footY, footZ], robot);
    if ~isValid
        joints = fallbackLegJoints(attachPoint, [footX, footY, footZ], robot);
    end

    set(hLegs(leg), 'XData', joints(:, 1), 'YData', joints(:, 2), 'ZData', joints(:, 3));
    set(hJoints(leg), 'XData', joints(:, 1), 'YData', joints(:, 2), 'ZData', joints(:, 3));
end
end

function joints = fallbackLegJoints(attachPoint, footPoint, robot)
deltaXY = footPoint(1:2) - attachPoint(1:2);
if norm(deltaXY) < eps
    dirXY = [1, 0];
else
    dirXY = deltaXY / norm(deltaXY);
end

coxaJoint = attachPoint + [dirXY * robot.L_coxa, 0];
kneeJoint = (coxaJoint + footPoint) / 2 + [0, 0, robot.L_femur * 0.4];
joints = [attachPoint; coxaJoint; kneeJoint; footPoint];
end
