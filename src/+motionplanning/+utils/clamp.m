function values = clamp(values, lowerBound, upperBound)
%CLAMP Restrict values to the closed interval [lowerBound, upperBound].

values = max(min(values, upperBound), lowerBound);
end
