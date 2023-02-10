%TEST_SPLINETRAJECTORY_COMPUTEPOLYNOMIAL tests the compute polynomial
%function of the SplineTrajectory class with CASADI varialbes.


% Coefficients, evaluation times, and order of derivatives to be computed
coeffs = casadi.SX.sym('coeffs', 4, 1);
computationTime = linspace(-3, 3, 100);
derivativeOrder = 3;

% Function call
p = SplineTrajectory.computePolynomial(coeffs, computationTime, derivativeOrder);

