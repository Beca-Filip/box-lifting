function [Forceplate] = ForceplateFilter(Forceplate,f_ech,f_cutoff, order)
%FORCEPLATEFILTER returns the forceplate data filtered by a filter with
%f_cutoff frequency and of given order, while f_ech is the sampling
%frequency.

% Get the markerset names
forceplateFieldnames = fieldnames(Forceplate);

% For each markerset
for numFieldname = 1 : length(forceplateFieldnames)
    
    % Get marker names
    Forceplate.(forceplateFieldnames{numFieldname}) = ...
    lowpass_filter(Forceplate.(forceplateFieldnames{numFieldname}).', f_ech, f_cutoff, order).';

end

