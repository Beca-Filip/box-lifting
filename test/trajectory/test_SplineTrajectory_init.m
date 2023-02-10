%TEST_SPLINETRAJECTORY_INIT tests a creation of an instance of the 
%SplineTrajectory class.

knotTimes = linspace(0, 1, 5);
knotValues = sin(2*pi*knotTimes);

s = SplineTrajectory(knotTimes, knotValues)