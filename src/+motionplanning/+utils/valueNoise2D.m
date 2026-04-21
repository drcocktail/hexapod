function noise = valueNoise2D(X, Y, frequency, bounds, octave)
%VALUENOISE2D Generate smooth value noise by interpolating a coarse lattice.

frequency = max(frequency, eps);
xCoords = unique(X(1, :));
if numel(xCoords) > 1
    gridSpacing = max(diff(xCoords));
else
    gridSpacing = 1;
end
featureSize = max(gridSpacing, 1 / frequency);

controlX = bounds.minX - featureSize:featureSize:bounds.maxX + featureSize;
controlY = bounds.minY - featureSize:featureSize:bounds.maxY + featureSize;

if numel(controlX) < 2
    controlX = [bounds.minX - featureSize, bounds.maxX + featureSize];
end
if numel(controlY) < 2
    controlY = [bounds.minY - featureSize, bounds.maxY + featureSize];
end

values = rand(numel(controlY), numel(controlX));
noise = interp2(controlX, controlY, values, X, Y, 'linear');

if any(isnan(noise(:)))
    replacement = mean(values(:));
    noise(isnan(noise)) = replacement;
end

% Slight octave-specific contrast avoids identical-looking low-frequency bands.
noise = noise - min(noise(:));
range = max(noise(:));
if range > 0
    noise = noise / range;
end
noise = noise .^ (1 + 0.05 * max(octave - 1, 0));
end
