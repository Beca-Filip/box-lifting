function [TableWidth, TableHeight, TableCenterCoordinates] = defaultTableParameters(obj)
%DEFAULTTABLEPARAMETERS returns the default table parameters for the lifting
%task.
%
%The numerical values are consistent with the experiment design.
%
%   [TableWidth, TableHeight, TableCenterCoordinates] = DEFAULTTABLEPARAMETERS(obj)
%   

% Default values
TableWidth = 0.4760;
TableHeight = 0.5870;
TableCenterCoordinates = [0.8130; 0.2940];
end

