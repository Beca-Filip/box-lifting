function obj = runConstantWeightGridIOC(obj, m_grid)
%RUNCONSTANTWEIGHTGRIDIOC implements the outer loop cost function for the weights w.

% Get number of w
nw = length(obj.doc.costFunctionVector);

% Get grid
w_grid = 100 * prob_simplex_ndim(nw-1, m_grid);

% Get RMSE
rmse_grid = zeros(1, size(w_grid, 1));

% Traverse grid
for gridPoint = 1 : length(w_grid)
    fprintf("Grid IOC Iter. %05d/%05d.\n", gridPoint, length(w_grid));
    
    % Calculate inner RMSE
    rmse_grid(gridPoint) = obj.costFunConstantWeight(w_grid(gridPoint, :).');
end

% Get solution
[minrmse, argminrmse] = min(rmse_grid);

% Set the solution
obj.solution.w = w_grid(argminrmse);
obj.solution.rmse = minrmse;
obj.solution.argmin_grid = argminrmse;
obj.solution.w_grid = w_grid;
obj.solution.rmse_grid = rmse_grid;
end