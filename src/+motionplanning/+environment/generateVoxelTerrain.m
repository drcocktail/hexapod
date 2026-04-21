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

zContinuous = zContinuous - min(zContinuous(:));
Z = round(zContinuous / options.blockHeight) * options.blockHeight;

distToDiagonal = abs(X - Y) / sqrt(2);
valleyMask = exp(-(distToDiagonal .^ 2) / (options.valleyInfluence ^ 2));
Z = max(0, Z - round(valleyMask * options.valleyDepth / options.blockHeight) * options.blockHeight);

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
