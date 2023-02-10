%TEST_SPLINETRAJECTORY_INIT tests a creation of an instance of the 
%SplineTrajectory class.

% Create knots
knotNumber = 10;
knotTimes = linspace(0, 1, knotNumber);
knotValues1 = sin(2*pi*knotTimes);
knotValues2 = cos(2*pi*knotTimes);
knotValues3 = log(1 + knotTimes);
knotValues = [knotValues1; knotValues2; knotValues3];

% Times and timestep at which the spline will be computed
computationTimes = linspace(0, 1, 100);
dt = computationTimes(2)-computationTimes(1);
% Compute the reference values
refValues1 = sin(2*pi*computationTimes);
refValues2 = cos(2*pi*computationTimes);
refValues3 = log(1 + computationTimes);
% Compute the derivative reference values
refDerivativeValues1 = diff(refValues1) ./ dt;
refDerivativeValues2 = diff(refValues2) ./ dt;
refDerivativeValues3 = diff(refValues3) ./ dt;

% Degree of interpolation
degree = 3;

% SplineBoundaryCondition constructor
Dimension = 3;
DerivativeOrder = 1;
Value1 = [refDerivativeValues1(1); refDerivativeValues2(1); refDerivativeValues3(1)];
Value2 = [refDerivativeValues1(end); refDerivativeValues2(end); refDerivativeValues3(end)];
KnotOfCondition1 = 1;
KnotOfCondition2 = knotNumber;

% boundary conditions
boundaryConditions = [ ...
                      SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition1, Value1);
                      SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition2, Value2);
                     ];
                 
% Create spline and interpolate
s = SplineTrajectory(knotTimes, knotValues, degree, boundaryConditions);
computeVals = s.computeValues(computationTimes, 1);

% Plot the comparison
figure;
subplot(3, 1, 1)
hold all
plot(computationTimes, refValues1, 'DisplayName', '$f_1$');
plot(computationTimes, computeVals{1}(1, :), 'DisplayName', '$s_1$');
legend('interpreter', 'latex', 'location', 'best');

subplot(3, 1, 2)
hold all
plot(computationTimes, refValues2, 'DisplayName', '$f_2$');
plot(computationTimes, computeVals{1}(2, :), 'DisplayName', '$s_2$');
legend('interpreter', 'latex', 'location', 'best');

subplot(3, 1, 3)
hold all
plot(computationTimes, refValues3, 'DisplayName', '$f_3$');
plot(computationTimes, computeVals{1}(3, :), 'DisplayName', '$s_3$');
legend('interpreter', 'latex', 'location', 'best');


% Plot the comparison
figure;
subplot(3, 1, 1)
hold all
plot(computationTimes(1:end-1), refDerivativeValues1, 'DisplayName', '$\partial f_1 / \partial t$');
plot(computationTimes, computeVals{2}(1, :), 'DisplayName', '$\partial s_1 / \partial t$');
legend('interpreter', 'latex', 'location', 'best');

subplot(3, 1, 2)
hold all
plot(computationTimes(1:end-1), refDerivativeValues2, 'DisplayName', '$\partial f_2 / \partial t$');
plot(computationTimes, computeVals{2}(2, :), 'DisplayName', '$\partial s_2 / \partial t$');
legend('interpreter', 'latex', 'location', 'best');

subplot(3, 1, 3)
hold all
plot(computationTimes(1:end-1), refDerivativeValues3, 'DisplayName', '$\partial f_3 / \partial t$');
plot(computationTimes, computeVals{2}(3, :), 'DisplayName', '$\partial s_3 / \partial t$');
legend('interpreter', 'latex', 'location', 'best');