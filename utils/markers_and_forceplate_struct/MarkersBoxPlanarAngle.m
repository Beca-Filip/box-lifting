function [boxAngle] = MarkersBoxPlanarAngle(Markers)
%MarkersBoxPlanarAngle extracts the angle of the box.

    % Extrapolate box information from the existing BOX markers
    % Box sides
    BOXDEPTH = Markers.BOX.FARDR - Markers.BOX.NEADR;
    
    % Angle
    boxAngle = atan2(BOXDEPTH(:, 2), BOXDEPTH(:, 1));
end        