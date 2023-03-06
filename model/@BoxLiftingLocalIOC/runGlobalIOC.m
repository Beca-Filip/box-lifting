function obj = runGlobalIOC(obj, varargin)
%RUNGLOBALIOC runs genetic algorithms for IOC.

% if nargin
if nargin > 1
    nT = varargin{1};
else
    nT = 3;
end

% pass optimoptions
if nargin > 2
    optimopts = varargin{2};
else
    optimopts = optimoptions(...
        @ga, ...
        'CreationFcn', 'gacreationlinearfeasible', ...
        'Display', 'iter', ...
        'FunctionTolerance', 1e-4, ...
        'PopulationSize', 200, ...
        'PlotFcn', 'gaplotbestf' ...
    );
end

% initialize weights
w0 = obj.getInitialWeightsIOC(length(obj.doc.costFunctionVector), nT);

% Get size of w0
[nCF, nT] = size(w0);

% $\sum_{j} w_{j, i} \geq 1$ 
% Inequality constraint matrix sum_w_geq_1
Asum_w_geq_1 = zeros(nT, nCF * nT);
for rr = 1 : nT
    Asum_w_geq_1(rr, (nCF - 1) * rr + 1 : nCF * rr) = -1;
end
% Inequality constraint vector sum_w_geq_1
bsum_w_geq_1 = - ones(nT, 1);

% $\sum_{j} w_{j, i} \leq 100$ 
% Inequality constraint matrix sum_w_leq_100
Asum_w_leq_100 = zeros(nT, nCF * nT);
for rr = 1 : nT
    Asum_w_leq_100(rr, (nCF - 1) * rr + 1 : nCF * rr) = 1;
end
% Inequality constraint vector sum_w_leq_100
bsum_w_leq_100 = 100 * ones(nT, 1);

% Stack into one matrix
A = vertcat(Asum_w_geq_1, Asum_w_leq_100);
b = vertcat(bsum_w_geq_1, bsum_w_leq_100);

% Put lower bounds on w
lb = zeros(nCF * nT, 1);
ub = 100 * ones(nCF * nT, 1);

% Reshape w
w0 = reshape(w0, [], 1);

% Call ga
[w_opt, rmse_opt, ef_opt, out_opt, population, scores] = ...
ga(@(w) obj.costFun(w), size(w0, 1), A, b, [], [], lb, ub, [], optimopts);

% Set the solution
obj.solution.w = w_opt;
obj.solution.rmse = rmse_opt;
obj.solution.exit_flag = ef_opt;
obj.solution.output = out_opt;
obj.solution.population = population;
obj.solution.scores = scores;
end