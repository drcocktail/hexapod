function smoothed = gaussianSmooth2D(values, sigma)
%GAUSSIANSMOOTH2D Smooth a 2-D array using separable base-MATLAB conv2.

if sigma <= 0
    smoothed = values;
    return;
end

radius = max(1, ceil(3 * sigma));
coords = -radius:radius;
kernel = exp(-(coords .^ 2) / (2 * sigma ^ 2));
kernel = kernel / sum(kernel);

smoothed = conv2(conv2(values, kernel, 'same'), kernel', 'same');
end
