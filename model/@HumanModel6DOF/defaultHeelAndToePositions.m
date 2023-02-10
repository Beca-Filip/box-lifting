function [HeelPosition,ToePosition] = defaultHeelAndToePositions(obj)
%DEFAULTHEELANDTOEPOSITIONS returns the 2D positions of the heel and of the
%heel in the base frame.
%   [HeelPosition,ToePosition] = DEFAULTHEELANDTOEPOSITIONS(obj)
%   Calculates them as the position of the foot center of mass with
%   subtracted and added half of the foot length.

% Length of half the foot along the X axis 
HalfFootLengthVector = [obj.LFOOT/2;0];

% Positions
HeelPosition = obj.CoMFOOT - HalfFootLengthVector;
ToePosition = obj.CoMFOOT + HalfFootLengthVector;

end
