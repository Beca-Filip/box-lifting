function [Markers] = MarkersTranslateRotate(p,R,Markers)
%MARKERSTRANSLATEROTATE first translates and then rotates the Markers 
%structure.

% reshape the position vector
p = reshape(p, [1, 3]);

% Get the markerset names
markersetNames = fieldnames(Markers);

% For each markerset
for numMarkerset = 1 : length(markersetNames)
    
    % Get marker names
    markerNames = fieldnames(Markers.(markersetNames{numMarkerset}));
    
    % For each marker
    for numMarker=  1 : length(markerNames)
        
        % Translate
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}) = ...
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}) + p;
    
        % Rotate
        Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}) = ...
        (R * Markers.(markersetNames{numMarkerset}).(markerNames{numMarker}).').';
    end
end
end

