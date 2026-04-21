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

fig = ax.Parent;
hPanel = uipanel('Parent', fig, 'Position', [0 0 1 0.08], 'Units', 'normalized', 'BackgroundColor', [0.1 0.1 0.15], 'BorderType', 'none');
btnPlayPause = uicontrol('Parent', hPanel, 'Style', 'pushbutton', 'String', 'Pause', ...
    'Units', 'normalized', 'Position', [0.35 0.2 0.08 0.6], 'Callback', @(~,~) togglePlayPause(fig));
btnRestart = uicontrol('Parent', hPanel, 'Style', 'pushbutton', 'String', 'Restart', ...
    'Units', 'normalized', 'Position', [0.44 0.2 0.08 0.6], 'Callback', @(~,~) triggerRestart(fig));
hText = uicontrol('Parent', hPanel, 'Style', 'text', 'String', 'Preparing...', ...
    'Units', 'normalized', 'Position', [0.53 0.2 0.15 0.6], 'ForegroundColor', 'w', 'BackgroundColor', [0.1 0.1 0.15], 'FontSize', 12);

setappdata(fig, 'playbackPlaying', true);
setappdata(fig, 'playbackRestart', false);

waypoint = 1;
frame = 1;

try
    while ishandle(ax)
        if getappdata(fig, 'playbackRestart')
            waypoint = 1;
            frame = 1;
            setappdata(fig, 'playbackRestart', false);
            setappdata(fig, 'playbackPlaying', true);
            set(btnPlayPause, 'String', 'Pause');
        end
        
        isPlaying = getappdata(fig, 'playbackPlaying');
        if ~isPlaying
            pause(0.1);
            continue;
        end
        
        if waypoint >= numWaypoints
            set(hText, 'String', 'Playback Finished - Reached Goal');
            setappdata(fig, 'playbackPlaying', false);
            set(btnPlayPause, 'String', 'Play');
            continue;
        end
        
        startXY = pathXY(waypoint, :);
        endXY = pathXY(waypoint + 1, :);
        startZ = pathZ(waypoint);
        endZ = pathZ(waypoint + 1);
        theta = atan2(endXY(2) - startXY(2), endXY(1) - startXY(1));
        
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
        set(hText, 'String', sprintf('Traversing Node: %d / %d', waypoint, numWaypoints));

        drawnow limitrate;
        pause(options.frameDelay);
        
        frame = frame + 1;
        if frame > options.framesPerWaypoint
            frame = 1;
            waypoint = waypoint + 1;
        end
    end
catch ME
    if ishandle(fig)
        rethrow(ME);
    end
end
end

function togglePlayPause(f)
    if ishandle(f)
        isPlaying = getappdata(f, 'playbackPlaying');
        setappdata(f, 'playbackPlaying', ~isPlaying);
        % The button text is refreshed dynamically by the main draw loop
        btn = gcbo;
        if ~isPlaying
            set(btn, 'String', 'Pause');
        else
            set(btn, 'String', 'Play');
        end
    end
end

function triggerRestart(f)
    if ishandle(f)
        setappdata(f, 'playbackRestart', true);
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
