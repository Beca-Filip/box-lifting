%TEST_SPLINETRAJECTORY_INIT tests a creation of an instance of the 
%SplineTrajectory class.

% Create knots
nbKnots = 20;
knotTimes = linspace(0, 1, nbKnots);
knotValues1 = casadi.SX.sym('knotValues1', 1, nbKnots);
knotValues2 = casadi.SX.sym('knotValues2', 1, nbKnots);
knotValues3 = casadi.SX.sym('knotValues3', 1, nbKnots);
knotValues = [knotValues1; knotValues2; knotValues3];

% Times and timestep at which the spline will be computed
nbEvalTimes = 100;
computationTimes = linspace(0, 1, nbEvalTimes);

% Create spline and interpolate
s = SplineTrajectory(knotTimes, knotValues);
% end
% toc
computeVals = s.computeValues(computationTimes, 1);