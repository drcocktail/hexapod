function terrain = generateVoxelTerrain(options)
%GENERATEVOXELTERRAIN Generate a blocky fBm-style terrain height field.

resolution = options.gridResolution;
domainSize = options.domainSize;

xVec = linspace(0, domainSize, resolution);
yVec = linspace(0, domainSize, resolution);
[X, Y] = meshgrid(xVec, yVec);

zContinuous = zeros(resolution, resolution);
frequency = options.frequency;
amplitude = options.amplitude;

for octave = 1:options.octaves
    octaveNoise = motionplanning.utils.valueNoise2D( ...
        X, Y, frequency, terrainBounds(domainSize), octave);
    zContinuous = zContinuous + octaveNoise * amplitude;

    amplitude = amplitude * options.persistence;
    frequency = frequency * 2;
end

zContinuous = zContinuous + randomTopologyFeatures(X, Y, options);
zContinuous = zContinuous - min(zContinuous(:));
maxTerrainHeight = getOption(options, 'maxTerrainHeight', Inf);
if isfinite(maxTerrainHeight) && max(zContinuous(:)) > maxTerrainHeight
    zContinuous = zContinuous * (maxTerrainHeight / max(zContinuous(:)));
end
Z = round(zContinuous / options.blockHeight) * options.blockHeight;

legacyValleyDepth = getOption(options, 'valleyDepth', 0);
valleyDepth = getOption(options, 'diagonalValleyDepth', legacyValleyDepth);
if valleyDepth > 0
    valleyInfluence = getOption(options, 'valleyInfluence', 6.0);
    distToDiagonal = abs(X - Y) / sqrt(2);
    valleyMask = exp(-(distToDiagonal .^ 2) / (valleyInfluence ^ 2));
    Z = max(0, Z - round(valleyMask * valleyDepth / options.blockHeight) * options.blockHeight);
end

terrain = struct( ...
    'X', X, ...
    'Y', Y, ...
    'Z', Z, ...
    'minX', 0, ...
    'maxX', domainSize, ...
    'minY', 0, ...
    'maxY', domainSize, ...
    'resolution', resolution, ...
    'xVec', xVec, ...
    'yVec', yVec, ...
    'gridSpacing', domainSize / max(resolution - 1, 1));
end

function bounds = terrainBounds(domainSize)
bounds = struct('minX', 0, 'maxX', domainSize, 'minY', 0, 'maxY', domainSize);
end

function features = randomTopologyFeatures(X, Y, options)
featureCount = getOption(options, 'topologyFeatureCount', 0);
features = zeros(size(X));
if featureCount <= 0
    return;
end

domainSize = options.domainSize;
heightScale = getOption(options, 'topologyFeatureHeight', 12.0);
minRadius = getOption(options, 'topologyMinRadius', domainSize * 0.03);
maxRadius = getOption(options, 'topologyMaxRadius', domainSize * 0.16);
faultStepHeight = getOption(options, 'faultStepHeight', heightScale * 0.6);

for idx = 1:featureCount %#ok<NASGU>
    centerX = rand() * domainSize;
    centerY = rand() * domainSize;
    radius = minRadius + rand() * (maxRadius - minRadius);
    height = heightScale * (0.35 + rand());
    if rand() < 0.45
        height = -height;
    end

    featureKind = rand();
    if featureKind < 0.25
        features = features + radialDome(X, Y, centerX, centerY, radius, height);
    elseif featureKind < 0.45
        features = features + ridgeOrCanyon(X, Y, centerX, centerY, radius, height);
    elseif featureKind < 0.65
        features = features + mesaOrSink(X, Y, centerX, centerY, radius, height);
    elseif featureKind < 0.82
        features = features + faultStep(X, Y, centerX, centerY, radius, faultStepHeight);
    else
        features = features + ringRidge(X, Y, centerX, centerY, radius, height);
    end
end
end

function feature = radialDome(X, Y, centerX, centerY, radius, height)
distSq = (X - centerX) .^ 2 + (Y - centerY) .^ 2;
feature = height * exp(-distSq / (2 * radius ^ 2));
end

function feature = ridgeOrCanyon(X, Y, centerX, centerY, radius, height)
theta = rand() * 2 * pi;
across = (X - centerX) * cos(theta) + (Y - centerY) * sin(theta);
along = -(X - centerX) * sin(theta) + (Y - centerY) * cos(theta);
width = max(radius * (0.12 + 0.18 * rand()), eps);
lengthScale = radius * (1.3 + 2.5 * rand());
feature = height * exp(-(across .^ 2) / (2 * width ^ 2)) .* ...
    exp(-(along .^ 2) / (2 * lengthScale ^ 2));
end

function feature = mesaOrSink(X, Y, centerX, centerY, radius, height)
dist = sqrt((X - centerX) .^ 2 + (Y - centerY) .^ 2);
edgeWidth = max(radius * 0.12, eps);
feature = height ./ (1 + exp((dist - radius) / edgeWidth));
end

function feature = faultStep(X, Y, centerX, centerY, radius, heightScale)
theta = rand() * 2 * pi;
signedDistance = (X - centerX) * cos(theta) + (Y - centerY) * sin(theta);
along = -(X - centerX) * sin(theta) + (Y - centerY) * cos(theta);
edgeWidth = max(radius * 0.08, eps);
lengthScale = radius * (1.5 + 3.0 * rand());
height = heightScale * (rand() - 0.5) * 2;
feature = 0.5 * height * tanh(signedDistance / edgeWidth) .* ...
    exp(-(along .^ 2) / (2 * lengthScale ^ 2));
end

function feature = ringRidge(X, Y, centerX, centerY, radius, height)
dist = sqrt((X - centerX) .^ 2 + (Y - centerY) .^ 2);
ringWidth = max(radius * (0.10 + 0.12 * rand()), eps);
feature = height * 0.65 * exp(-((dist - radius) .^ 2) / (2 * ringWidth ^ 2));
feature = feature - height * 0.35 * exp(-(dist .^ 2) / (2 * (radius * 0.55) ^ 2));
end

function value = getOption(options, fieldName, defaultValue)
if isfield(options, fieldName)
    value = options.(fieldName);
else
    value = defaultValue;
end
end
