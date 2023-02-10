function [L] = MarkersGetSegmentLengths(Markers)
%MARKERSGETSEGMENTLENGTHS calculates the 6DOF segment legnths from
%markers.

L1 = sqrt(sum((Markers.BODY.RKNE - Markers.BODY.RANK).^2, 2));
L2 = sqrt(sum((Markers.BODY.RGTR - Markers.BODY.RKNE).^2, 2));
L3 = sqrt(sum((Markers.BODY.BACK - Markers.BODY.RGTR).^2, 2));
L4 = sqrt(sum((Markers.BODY.RSHO - Markers.BODY.BACK).^2, 2));
L5 = sqrt(sum((Markers.BODY.RELB - Markers.BODY.RSHO).^2, 2));
L6 = sqrt(sum((Markers.BODY.RWRI - Markers.BODY.RELB).^2, 2));

L = [L1, L2, L3, L4, L5, L6];
end

