function [knotTimes, knotValues] = getEquidistantSplineKnots(q, t, numKnots)
    
    % Get the number of samples
    nbSamples = size(q, 2);
    
    % Get the indices
    equidistantIndices = linspace(1, nbSamples, numKnots);
    
    % Get the floor and ceil of these indices
    floorEquidistantIndices = floor(equidistantIndices);
    ceilEquidistantIndices = ceil(equidistantIndices);
    
    % For all indices, get offset from the floor indices
    alphaEquidistantIndices = equidistantIndices - floorEquidistantIndices;
    
    % For integer indices this will just pick them out
    % For non-integer indices this will linearly interpolate between them
    knotTimes = (1 - alphaEquidistantIndices) .* t(floorEquidistantIndices) + alphaEquidistantIndices .* t(ceilEquidistantIndices);
    knotValues = (1 - alphaEquidistantIndices) .* q(:, floorEquidistantIndices) + alphaEquidistantIndices .* q(:, ceilEquidistantIndices);
    
end