function [WristInitialPosition, WristFinalPosition] = defaultWristPositionParameters(obj)
%DEFAULTWRISTPOSITIONPARAMETERS returns the default wrist position parameters for the lifting
%task.
%
%The numerical values are consistent with the experiment design.
%
%   [WristInitialPosition, WristFinalPosition] = DEFAULTWRISTPOSITIONPARAMETERS(obj)
%   

% Default values
WristInitialPosition = [0.4903; 0.2952];
WristFinalPosition = [0.6557; 0.9343];

end

