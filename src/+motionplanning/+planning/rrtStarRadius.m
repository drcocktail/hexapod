function radius = rrtStarRadius(options, nodeCount)
%RRTSTARRADIUS Return the asymptotic RRT* neighborhood radius in 2-D.

if nodeCount <= 1
    radius = options.rewireRadius;
    return;
end

gamma = options.rrtStarGamma;
radius = min(options.rewireRadius, gamma * sqrt(log(nodeCount) / nodeCount));
radius = max(radius, options.stepSize * 1.5);
end
