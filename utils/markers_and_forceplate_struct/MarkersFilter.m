function [Markers] = MarkersFilter(Markers,f_ech,f_cutoff,order)
%MarkersFilter returns the Markers data filtered by a filter with
%f_cutoff frequency and of given order, while f_ech is the sampling
%frequency.

% Get the markerset names
markersetNames = fieldnames(Markers);

% For each markerset
for numMarkerset = 1 : length(markersetNames)
    
    % Get marker names
    markerNames = fieldnames(Markers.(markersetNames{numMarkerset}));
    
    % For each marker
    for numMarker = 1 : length(markerNames)
        
        % Translate
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}) = ...
        lowpass_filter(Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}).', f_ech, f_cutoff, order).';
    
    end
end
end

