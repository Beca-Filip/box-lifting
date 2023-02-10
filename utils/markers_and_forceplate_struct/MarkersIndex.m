function [Markers] = MarkersIndex(Markers,indices)
%MARKERSINDEX returns the markers given by the indices.

% Get the markerset names
markersetNames = fieldnames(Markers);

% For each markerset
for numMarkerset = 1 : length(markersetNames)
    
    % Get marker names
    markerNames = fieldnames(Markers.(markersetNames{numMarkerset}));
    
    % For each marker
    for numMarker=  1 : length(markerNames)
        
        % Index
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}) = ...
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker})(indices, :);
    
    end
end

end

