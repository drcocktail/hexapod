% Hexapod 3D Morphological Simulator with Volumetric Planning
% Architecture: Procedural Voxel Terrain, W-Space Minkowski Expansion, and 3D Kinematics

function Hexapod_Simulator()
    clear; clc; close all;
    
    %% MODULE 1: CONFIGURATION AND SEEDING
    fprintf('Initializing Hexapod Volumetric Simulator...\n');
    
    % Generate a random seed for procedural generation, but record it
    simSeed = randi(10000);
    rng(simSeed);
    fprintf('Procedural Generation Seed: %d\n', simSeed);
    
    % Robot Morphological Parameters
    robot.bodyRadius = 4.0;
    robot.bodyHeight = 2.0;  % Thickness of the 3D chassis
    robot.clearance  = 3.5;  % Target distance from belly to max terrain under body
    robot.L_coxa     = 1.5;
    robot.L_femur    = 4.5;
    robot.L_tibia    = 5.5;
    robot.maxReach   = robot.L_coxa + robot.L_femur + robot.L_tibia - 1.0;
    robot.maxStepZ   = 6.0;  % Maximum vertical kinematic limit
    
    % Hexagonal symmetry for leg attachments
    robot.baseAngles = linspace(0, 2*pi, 7); 
    robot.baseAngles(end) = [];
    
    %% MODULE 2: PROCEDURAL VOXEL TERRAIN GENERATION
    fprintf('Generating Voxelized fBm Terrain...\n');
    gridResolution = 80;
    domainSize = 60;
    [X, Y, Z, terrain] = generateVoxelTerrain(gridResolution, domainSize);
    
    % Define Start and Goal (Ensure they are placed in valid regions)
    startNode = [5, 5];
    goalNode  = [domainSize-5, domainSize-5];
    
    % Flatten start and goal regions to guarantee kinematic feasibility at spawn
    Z = flattenRegion(Z, X, Y, startNode, robot.bodyRadius * 1.5);
    Z = flattenRegion(Z, X, Y, goalNode, robot.bodyRadius * 1.5);
    terrain.Z = Z; % Update terrain object
    
    %% MODULE 3: PATH PLANNING (VOLUMETRIC EVALUATION)
    fprintf('Executing Minkowski-Expanded RRT* Planning...\n');
    [pathWSpace, pathZ] = planVolumetricPath(startNode, goalNode, terrain, robot);
    
    if isempty(pathWSpace)
        error('Solver failed to find a valid volumetric path. The environment may be completely obstructed. Re-run for a new procedural seed.');
    end
    
    %% MODULE 4: 3D VISUALIZATION ENGINE
    fprintf('Rendering 3D Environment...\n');
    fig = figure('Name', sprintf('Hexapod Simulator (Seed: %d)', simSeed), 'Color', [0.1 0.1 0.15], 'Position', [50, 50, 1200, 900]);
    ax = axes('Parent', fig, 'Color', [0.1 0.1 0.15]);
    hold(ax, 'on'); grid(ax, 'on'); view(ax, [-45, 45]);
    
    % Configure lighting for blocky geometry
    light(ax, 'Position', [0, 0, 100], 'Style', 'local', 'Color', [0.9 0.9 1.0]);
    light(ax, 'Position', [domainSize, domainSize, 50], 'Style', 'local', 'Color', [1.0 0.8 0.6]);
    lighting(ax, 'flat'); % Flat lighting emphasizes the voxel/Minecraft aesthetic
    
    % Render Voxel Terrain using surface (approximating voxels for performance)
    surf(ax, terrain.X, terrain.Y, terrain.Z, 'EdgeColor', [0.2 0.2 0.2], 'EdgeAlpha', 0.3, 'FaceColor', [0.3 0.4 0.3]);
    
    % Plot Path
    plot3(ax, pathWSpace(:,1), pathWSpace(:,2), pathZ, 'w--', 'LineWidth', 2);
    plot3(ax, startNode(1), startNode(2), pathZ(1), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(ax, goalNode(1), goalNode(2), pathZ(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    axis(ax, 'equal');
    axis(ax, [0 domainSize 0 domainSize 0 25]);
    xlabel(ax, 'X Axis', 'Color', 'w'); 
    ylabel(ax, 'Y Axis', 'Color', 'w'); 
    zlabel(ax, 'Elevation', 'Color', 'w');
    set(ax, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    
    %% MODULE 5: 3D KINEMATIC ANIMATION
    fprintf('Executing 3D Kinematic Animation...\n');
    animate3DHexapod(ax, pathWSpace, terrain, robot);
    
    fprintf('Simulation Execution Completed.\n');
end

%% ========================================================================
%% LOCAL FUNCTIONS AND MODULE IMPLEMENTATIONS
%% ========================================================================

%% --- Terrain Generation Module ---
function [X, Y, Z, terrain] = generateVoxelTerrain(resolution, size)
    % Generates a blocky, Minecraft-style terrain using Fractional Brownian Motion
    x_vec = linspace(0, size, resolution);
    y_vec = linspace(0, size, resolution);
    [X, Y] = meshgrid(x_vec, y_vec);
    
    % Fractal Noise Synthesis (Multiple Octaves)
    Z_continuous = zeros(resolution, resolution);
    octaves = 4;
    persistence = 0.5;
    frequency = 0.05;
    amplitude = 12.0;
    
    for o = 1:octaves
        % Generate a random grid for this octave
        randGrid = rand(resolution, resolution);
        % Smooth the grid
        smoothed = imgaussfilt(randGrid, 1 / frequency);
        Z_continuous = Z_continuous + smoothed * amplitude;
        
        amplitude = amplitude * persistence;
        frequency = frequency * 2;
    end
    
    % Normalize and scale
    Z_continuous = Z_continuous - min(Z_continuous(:));
    
    % Voxelization: Quantize the height to create discrete blocks
    blockHeight = 2.0;
    Z = round(Z_continuous / blockHeight) * blockHeight;
    
    % Carve a rough diagonal valley to ensure a path exists most of the time
    valleyInfluence = 6.0;
    distToDiagonal = abs(X - Y) / sqrt(2);
    valleyMask = exp(-(distToDiagonal.^2) / (valleyInfluence^2));
    Z = max(0, Z - round(valleyMask * 10 / blockHeight) * blockHeight);
    
    terrain.X = X;
    terrain.Y = Y;
    terrain.Z = Z;
    terrain.minX = 0; terrain.maxX = size;
    terrain.minY = 0; terrain.maxY = size;
    terrain.resolution = resolution;
end

function Z = flattenRegion(Z, X, Y, center, radius)
    % Forces a region to be flat (used for start and goal zones)
    distSq = (X - center(1)).^2 + (Y - center(2)).^2;
    mask = distSq < radius^2;
    if any(mask(:))
        flatHeight = min(Z(mask));
        Z(mask) = flatHeight;
    end
end

function z = getTerrainHeight(x, y, terrain)
    % Interpolates discrete terrain height at continuous coordinates
    x = max(min(x, terrain.maxX), terrain.minX);
    y = max(min(y, terrain.maxY), terrain.minY);
    z = interp2(terrain.X, terrain.Y, terrain.Z, x, y, 'nearest', 0);
end

%% --- Collision and Planning Module ---
function [isValid, reqBodyZ] = checkVolumetricState(x, y, terrain, robot)
    % Evaluates the W-Space Minkowski expansion.
    % The robot body is treated as a cylinder. We find the max terrain height
    % within the body radius.
    
    % 1. Body Collision Check
    % Define bounding box for fast query
    idxX = terrain.X(1,:) >= (x - robot.bodyRadius) & terrain.X(1,:) <= (x + robot.bodyRadius);
    idxY = terrain.Y(:,1) >= (y - robot.bodyRadius) & terrain.Y(:,1) <= (y + robot.bodyRadius);
    
    localX = terrain.X(idxY, idxX);
    localY = terrain.Y(idxY, idxX);
    localZ = terrain.Z(idxY, idxX);
    
    % Exact circular mask
    distSq = (localX - x).^2 + (localY - y).^2;
    mask = distSq <= robot.bodyRadius^2;
    
    if any(mask(:))
        maxZ_under_body = max(localZ(mask));
    else
        maxZ_under_body = getTerrainHeight(x, y, terrain);
    end
    
    % The body must sit above the highest point beneath it
    reqBodyZ = maxZ_under_body + robot.clearance;
    
    % 2. Kinematic Reachability Check
    % Ensure that from this reqBodyZ, all 6 legs can reach the actual terrain
    isValid = true;
    for leg = 1:6
        angle = robot.baseAngles(leg);
        % Approximate optimal foot placement
        footX = x + (robot.bodyRadius + robot.L_coxa + robot.L_femur * 0.7) * cos(angle);
        footY = y + (robot.bodyRadius + robot.L_coxa + robot.L_femur * 0.7) * sin(angle);
        
        footZ = getTerrainHeight(footX, footY, terrain);
        verticalDrop = reqBodyZ - footZ;
        
        if verticalDrop > robot.maxReach || verticalDrop < 0
            isValid = false;
            return;
        end
    end
end

function [path, pathZ] = planVolumetricPath(startPos, goalPos, terrain, robot)
    % Rapidly Exploring Random Tree (RRT) optimized for Volumetric Constraints
    maxNodes = 1500;
    stepSize = 3.0;
    
    nodes = zeros(maxNodes, 2);
    nodesZ = zeros(maxNodes, 1);
    parents = zeros(maxNodes, 1);
    
    nodes(1, :) = startPos;
    [~, startZ] = checkVolumetricState(startPos(1), startPos(2), terrain, robot);
    nodesZ(1) = startZ;
    nodeCount = 1;
    
    goalReached = false;
    goalIdx = -1;
    
    for i = 1:maxNodes
        % Goal Bias
        if rand() < 0.15
            q_rand = goalPos;
        else
            q_rand = [rand() * terrain.maxX, rand() * terrain.maxY];
        end
        
        % Nearest Neighbor
        dists = sum((nodes(1:nodeCount, :) - q_rand).^2, 2);
        [~, nearestIdx] = min(dists);
        q_near = nodes(nearestIdx, :);
        
        % Steer
        dir = q_rand - q_near;
        dist = norm(dir);
        if dist == 0; continue; end
        q_new = q_near + min(stepSize, dist) * (dir / dist);
        
        % Volumetric and Kinematic Validation
        [isValid, newZ] = checkVolumetricState(q_new(1), q_new(2), terrain, robot);
        
        if isValid
            % Edge gradient check (prevent scaling sheer cliffs)
            if abs(newZ - nodesZ(nearestIdx)) <= robot.maxStepZ
                nodeCount = nodeCount + 1;
                nodes(nodeCount, :) = q_new;
                nodesZ(nodeCount) = newZ;
                parents(nodeCount) = nearestIdx;
                
                % Check goal condition
                if norm(q_new - goalPos) < stepSize
                    goalReached = true;
                    goalIdx = nodeCount;
                    break;
                end
            end
        end
    end
    
    if ~goalReached
        path = []; pathZ = [];
        return;
    end
    
    % Backtrack
    path = goalPos;
    [~, finalZ] = checkVolumetricState(goalPos(1), goalPos(2), terrain, robot);
    pathZ = finalZ;
    
    currIdx = goalIdx;
    while currIdx > 1
        path = [nodes(currIdx, :); path]; %#ok<AGROW>
        pathZ = [nodesZ(currIdx); pathZ]; %#ok<AGROW>
        currIdx = parents(currIdx);
    end
    path = [startPos; path];
    pathZ = [nodesZ(1); pathZ];
end

%% --- 3D Visualization and Kinematics Module ---
function animate3DHexapod(ax, path, terrain, robot)
    % Renders a complex 3D morphological model of the robot navigating the path
    
    % 1. Construct 3D Chassis Prototype (Hexagonal Prism)
    t = robot.baseAngles;
    chassisTop = [robot.bodyRadius*cos(t), robot.bodyRadius*sin(t),  ones(6,1)*(robot.bodyHeight/2)];
    chassisBot = [robot.bodyRadius*cos(t), robot.bodyRadius*sin(t), -ones(6,1)*(robot.bodyHeight/2)];
    
    % Define Patch Faces for the Chassis
    faces = [
        1 2 3 4 5 6; % Top
        7 8 9 10 11 12; % Bottom
        1 2 8 7 NaN NaN; % Sides
        2 3 9 8 NaN NaN;
        3 4 10 9 NaN NaN;
        4 5 11 10 NaN NaN;
        5 6 12 11 NaN NaN;
        6 1 7 12 NaN NaN
    ];
    
    hChassis = patch(ax, 'Faces', faces, 'Vertices', [chassisTop; chassisBot], ...
        'FaceColor', [0.15 0.25 0.35], 'EdgeColor', [0.0 0.8 0.8], 'LineWidth', 1.5);
    
    % 2. Initialize 3D Articulated Legs
    hLegs = gobjects(1, 6);
    hJoints = gobjects(1, 6);
    for i = 1:6
        % Thick lines for leg segments
        hLegs(i) = plot3(ax, zeros(1,4), zeros(1,4), zeros(1,4), ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 4);
        % Spheres for joints
        hJoints(i) = plot3(ax, zeros(1,4), zeros(1,4), zeros(1,4), ...
            'o', 'MarkerSize', 6, 'MarkerFaceColor', [1 0.4 0], 'MarkerEdgeColor', 'k');
    end
    
    numWaypoints = size(path, 1);
    framesPerWaypoint = 12;
    
    % Pre-compute all volumetric Z heights for smoothness
    zHeights = zeros(numWaypoints, 1);
    for w = 1:numWaypoints
        [~, zHeights(w)] = checkVolumetricState(path(w,1), path(w,2), terrain, robot);
    end
    
    for w = 1:(numWaypoints-1)
        pStart = path(w, :);
        pEnd = path(w+1, :);
        zStart = zHeights(w);
        zEnd = zHeights(w+1);
        
        % Calculate Heading
        theta = atan2(pEnd(2) - pStart(2), pEnd(1) - pStart(1));
        
        for f = 1:framesPerWaypoint
            alpha = f / framesPerWaypoint;
            
            % Smooth interpolation (cosine easing)
            ease = (1 - cos(alpha * pi)) / 2;
            
            currX = pStart(1) + ease * (pEnd(1) - pStart(1));
            currY = pStart(2) + ease * (pEnd(2) - pStart(2));
            currZ = zStart + ease * (zEnd - zStart);
            
            % Compute Pitch and Roll based on underlying terrain
            % Sample points ahead, behind, left, and right
            zFwd = getTerrainHeight(currX + robot.bodyRadius*cos(theta), currY + robot.bodyRadius*sin(theta), terrain);
            zBck = getTerrainHeight(currX - robot.bodyRadius*cos(theta), currY - robot.bodyRadius*sin(theta), terrain);
            zLft = getTerrainHeight(currX + robot.bodyRadius*cos(theta+pi/2), currY + robot.bodyRadius*sin(theta+pi/2), terrain);
            zRgt = getTerrainHeight(currX + robot.bodyRadius*cos(theta-pi/2), currY + robot.bodyRadius*sin(theta-pi/2), terrain);
            
            pitch = atan2(zFwd - zBck, 2*robot.bodyRadius) * 0.5; % Dampened pitch
            roll  = atan2(zRgt - zLft, 2*robot.bodyRadius) * 0.5; % Dampened roll
            
            % Rotation Matrices
            R_yaw = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
            R_pitch = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            R_roll = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            R_total = R_yaw * R_pitch * R_roll;
            
            % Transform Chassis
            baseVerts = [chassisTop; chassisBot];
            rotVerts = (R_total * baseVerts')';
            transVerts = rotVerts + repmat([currX, currY, currZ], 12, 1);
            set(hChassis, 'Vertices', transVerts);
            
            % Compute Leg Inverse Kinematics
            for leg = 1:6
                baseAngle = robot.baseAngles(leg);
                
                % Joint 1: Coxa attachment (extract from transformed chassis)
                attachVert = transVerts(leg, :); % Use top vertices for attachment
                
                % Determine resting foot target (in global coordinates)
                legHeading = theta + baseAngle;
                targetDist = robot.bodyRadius + robot.L_coxa + robot.L_femur * 0.6;
                footX = currX + targetDist * cos(legHeading);
                footY = currY + targetDist * sin(legHeading);
                footZ = getTerrainHeight(footX, footY, terrain);
                
                % Simulate stepping motion (lift legs alternately)
                isLifting = mod(leg + round(alpha*2), 2) == 0;
                if isLifting
                    footZ = footZ + 1.5 * sin(alpha * pi); % Parabolic step trajectory
                end
                
                % Joint 2: Femur connection
                dirVec = [footX - attachVert(1), footY - attachVert(2), 0];
                dirVec = dirVec / norm(dirVec);
                femurJoint = attachVert + dirVec * robot.L_coxa;
                
                % Joint 3: Knee calculation
                % Simplified IK: Place knee above the midpoint between femur joint and foot
                midX = (femurJoint(1) + footX) / 2;
                midY = (femurJoint(2) + footY) / 2;
                midZ = (femurJoint(3) + footZ) / 2 + robot.L_femur * 0.4; % Arch the knee up
                kneeJoint = [midX, midY, midZ];
                
                % Update Graphics
                pX = [attachVert(1), femurJoint(1), kneeJoint(1), footX];
                pY = [attachVert(2), femurJoint(2), kneeJoint(2), footY];
                pZ = [attachVert(3), femurJoint(3), kneeJoint(3), footZ];
                
                set(hLegs(leg), 'XData', pX, 'YData', pY, 'ZData', pZ);
                set(hJoints(leg), 'XData', pX, 'YData', pY, 'ZData', pZ);
            end
            
            drawnow limitrate;
            pause(0.015);
        end
    end
end