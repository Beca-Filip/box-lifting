%TEST_SPLINETRAJECTORY_COMPUTEPOLYNOMIAL tests the compute polynomial
%function of the SplineTrajectory class.


% Coefficients, evaluation times, and order of derivatives to be computed
coeffs = [0, 0, 0, 1];
computationTime = linspace(-3, 3, 100);
derivativeOrder = 3;

% Function call
p = SplineTrajectory.computePolynomial(coeffs, computationTime, derivativeOrder);

% Plotting
figure;
hold all;
% Colormap for the curves
cmap = linspecer(1 + derivativeOrder);
for d = 1 : derivativeOrder + 1
    plot(computationTime, p(d, :), 'DisplayName', sprintf("$p^{(%d)}(x)$", d-1), 'Color', cmap(d, :));
end
legend('interpreter', 'latex');