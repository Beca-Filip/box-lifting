function [Forceplate] = ForceplateIndex(Forceplate,indices)
%FORCEPLATEINDEX returns the forceplate data given by the indices.

% Get the markerset names
forceplateFieldnames = fieldnames(Forceplate);

% For each markerset
for numFieldname = 1 : length(forceplateFieldnames)
    
    % Get marker names
    Forceplate.(forceplateFieldnames{numFieldname}) = ...
    Forceplate.(forceplateFieldnames{numFieldname})(indices, :);

end

