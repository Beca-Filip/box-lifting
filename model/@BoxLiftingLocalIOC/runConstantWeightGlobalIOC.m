function obj = runConstantWeightGlobalIOC(obj, varargin)
%RUNCONSTANTWEIGHTGLOBALIOC runs genetic algorithms for IOC.

% pass optimoptions
if nargin > 1
    optimopts = varargin{1};
else
    optimopts = optimoptions(...
        @ga, ...
        'CreationFcn', 'gacreationlinearfeasible', ...
        'Display', 'iter', ...
        'FunctionTolerance', 2.6e-3, ...
        'PopulationSize', 25, ...
        'PlotFcn', 'gaplotbestf' ...
    );
end

% Get number of w
nw = length(obj.doc.costFunctionVector);

% We only want the sum to be greater than 1
A = -ones(1, nw);
b = -1;

% Put lower and upper bounds on w
lb = zeros(nw, 1);
ub = 1e4 * ones(nw, 1);

% Call ga
[w_opt, rmse_opt, ef_opt, out_opt, population, scores] = ...
ga(@(w) obj.costFunConstantWeight_noInit(w), nw, A, b, [], [], lb, ub, [], optimopts);

% Set the solution
obj.solution.w = w_opt;
obj.solution.rmse = rmse_opt;
obj.solution.exit_flag = ef_opt;
obj.solution.output = out_opt;
obj.solution.population = population;
obj.solution.scores = scores;
end