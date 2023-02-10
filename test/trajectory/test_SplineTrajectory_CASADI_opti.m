%TEST_SPLINETRAJECTORY_CASADI_OPTI creates a casadi.Opti object and
%initializes the SplineTrajectory with the default function.

opti = casadi.Opti();

% Spline trajectory parameters
dimension = 6;
degree = 5;
knotNumber = 10;
evaluationNumber = 100;
evaluationTimes = opti.parameter(1, evaluationNumber);

[trajectory, opti] = SplineTrajectory.defaultCasadiOptiInitialize(opti, dimension, degree, knotNumber, evaluationNumber);

V = trajectory.computeValues(evaluationTimes, degree);