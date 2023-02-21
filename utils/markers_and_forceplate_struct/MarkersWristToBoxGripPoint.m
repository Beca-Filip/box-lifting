function [WristToBoxGripPoint] = MarkersWristToBoxGripPoint(Markers)
%MARKERSWRISTTOBOXGRIPPOINT extracts the wrist to box grip point vector.

    % Extrapolate box information from the existing BOX markers
    % Box sides
    BOXVERTICAL = Markers.BOX.FARUR - Markers.BOX.FARDR;
%     BOXHORIZONTAL = Markers.BOX.FARDL - Markers.BOX.FARDR;
    BOXDEPTH = Markers.BOX.FARDR - Markers.BOX.NEADR;
    % BOX structure containing BOX points information
    BOX = Markers.BOX;
    % Compute other BOX points with given side information
    BOX.NEAUR = ((BOX.NEADR + BOXVERTICAL) + (BOX.FARUR - BOXDEPTH)) / 2;       % Near upper right point (To have all four points on the right side)
    BOX.CENTERR = ((BOX.NEADR + BOX.NEAUR + BOX.FARUR + BOX.FARDR)) / 2;        % Center-right point (For collisions)
    BOX.GRIPR = (BOX.NEAUR + BOX.FARUR) / 2;                                    % Grip point on right side (For external wrenches transfer)
    
    % Return vector
    WristToBoxGripPoint = BOX.GRIPR - Markers.BODY.RWRI;
end        