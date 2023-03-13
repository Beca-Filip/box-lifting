function CorrMarkers = MarkersS1CorrectLabelling(Markers, LFOOT, BOXHEIGHT)
%MARKERSS1CORRECTLABELLING corrects the labeling for S1 markers.

% Keep labels for existing markers on BODY
CorrMarkers.BODY.BACK = Markers.BODY.BACK;
% Change labels for existing but differently-named markers on BODY
CorrMarkers.BODY.RANK = Markers.BODY.ANKLE;
CorrMarkers.BODY.RELB = Markers.BODY.ELBOW;
CorrMarkers.BODY.RGTR = Markers.BODY.GTROC;
CorrMarkers.BODY.RHEE = Markers.BODY.HEEL;
CorrMarkers.BODY.RKNE = Markers.BODY.KNEE;
CorrMarkers.BODY.RSHO = Markers.BODY.SHOULDER;
CorrMarkers.BODY.RWRI = Markers.BODY.WRIST;

% Change labels for existing but differently-named markers on BOX
CorrMarkers.BOX.FARDR = Markers.BOX.FARR;
CorrMarkers.BOX.NEADR = Markers.BOX.NEAR;

% Keep labels for existing markers on TABLE
CorrMarkers.TABLE.FARL = Markers.TABLE.FARL;
CorrMarkers.TABLE.FARR = Markers.TABLE.FARR;
CorrMarkers.TABLE.NEAR = Markers.TABLE.NEAR;

% Compute labels for non-existing markers on BODY from the existing ones
% RTOE marker: HEEL marker + LFOOT * (Unit Vector from Hell to Metatarsal)
HEEL_TO_METATARSAL_DIRECTION = (Markers.BODY.METATARSAL - Markers.BODY.HEEL) ./ sqrt(sum((Markers.BODY.METATARSAL - Markers.BODY.HEEL).^2, 2));
CorrMarkers.BODY.RTOE = Markers.BODY.HEEL + HEEL_TO_METATARSAL_DIRECTION * LFOOT;

% Compute labels for non-existing markers on BOX from the existing ones

% Unit Vector for Box Vertical Side = cross(FARR-NEAR, FARMID-FARR) / norm(cross(FARR-NEAR, FARMID-FARR))
% FARUR marker: FARDR marker + BOXHEIGHT * (Unit Vector for Box Vertical Side)
BOX_VERTICAL_DIRECTION = cross(Markers.BOX.FARR - Markers.BOX.NEAR, Markers.BOX.FARMID - Markers.BOX.FARR) ./ ...
                sqrt(sum(cross(Markers.BOX.FARR - Markers.BOX.NEAR, Markers.BOX.FARMID - Markers.BOX.FARR).^2, 2));
CorrMarkers.BOX.FARUR = Markers.BOX.FARR + BOX_VERTICAL_DIRECTION * BOXHEIGHT;

% b = FARMID-FARR; a = FARR-NEAR;
% Half Width Vector for Box Horizontal Side = Projection of FARMID-FARR on perpendicular of FARR-NEAR = b - (b.' * a) * a
% FARDL marker: FARDR marker + 2 * (Half Width Vector for Box Horizontal Side)
b = (Markers.BOX.FARMID - Markers.BOX.FARR).';
a = (Markers.BOX.FARR - Markers.BOX.NEAR).';
HALF_BOX_WIDTH = (b - dot(b, a) .* a).';
CorrMarkers.BOX.FARDL = Markers.BOX.FARR +  2 * HALF_BOX_WIDTH;

% Reorder fields
CorrMarkers.BODY = orderfields(CorrMarkers.BODY);
CorrMarkers.BOX = orderfields(CorrMarkers.BOX);
CorrMarkers.TABLE = orderfields(CorrMarkers.TABLE);
end        