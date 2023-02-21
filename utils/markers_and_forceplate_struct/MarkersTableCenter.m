function [TableCenter, TableWidth, TableHeight] = MarkersTableCenter(Markers)
%MARKERSTABLECENTER extracts the table information.

    % Extrapolate table information from the existing TABLE markers
%     TABLEHORIZONTAL = Markers.TABLE.FARL - Markers.TABLE.FARR;
    TABLEDEPTH = Markers.TABLE.FARR - Markers.TABLE.NEAR;
    
    % Extract Width and Height
    TableWidth = norm(mean(TABLEDEPTH));
    TableHeight = mean(Markers.TABLE.FARR(:, 2) + Markers.TABLE.NEAR(:, 2) + Markers.TABLE.FARL(:, 2)) / 3;
    
    % Table center
    TableCenter = (Markers.TABLE.FARR + Markers.TABLE.NEAR) / 2;
    TableCenter(:, 2) = TableCenter(:, 2) - TableHeight / 2;
    TableCenter = mean(TableCenter);
    TableCenter = TableCenter(1:2);
end        