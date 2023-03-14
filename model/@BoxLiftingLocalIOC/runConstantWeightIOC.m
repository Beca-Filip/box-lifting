function obj = runConstantWeightIOC(obj, varargin)
%RUNCONSTANTWEIGHTIOC implements the outer loop cost function for the weights w.

% pass optimoptions
if nargin > 1
    optimopts = varargin{1};
else
    optimopts = optimoptions(...
        @fmincon, ...
        'Algorithm', 'interior-point', ...
        'Display', 'iter-detailed', ...
        'MaxIterations', 120, ...
        'PlotFcn', {'optimplotfval', 'optimplotstepsize'} ...
    );
end

% Get number of w
nw = length(obj.doc.costFunctionVector);

% initialize weights
w0 = 100 * ones(length(obj.doc.costFunctionVector), 1);

% We only want the sum to be greater than 1
A = -ones(1, nw);
b = -1;

% Put lower and upper bounds on w
lb = zeros(nw, 1);
ub = 1e4 * ones(nw, 1);

% Call fmincon
[w_opt, rmse_opt, ef_opt, out_opt, lambda_opt, grad_opt, hessian_opt] = ...
fmincon(@(w) obj.costFunConstantWeight(w), w0, A, b, [], [], lb, ub, [], optimopts);

% Set the solution
obj.solution.w = w_opt;
obj.solution.rmse = rmse_opt;
obj.solution.exit_flag = ef_opt;
obj.solution.output = out_opt;
obj.solution.lambda = lambda_opt;
obj.solution.grad = grad_opt;
obj.solution.hessian = hessian_opt;
end