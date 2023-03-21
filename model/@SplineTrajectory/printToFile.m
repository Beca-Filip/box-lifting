function printToFile(obj, filename)

% Get dimension
n = obj.dimension;
% Compute derivatives at knot times
derivVals = obj.computeValues(obj.knotTimes, obj.degree-1);

% Prealocate variables that are going in the table columns
var = cell(1 + 2*n, 1);
% First column
var{1} = obj.knotTimes.';
% Other columns
for ii = 1 : n
    % Get knot values for iith dimension
    var{ii+1} = obj.knotValues(ii, :).';
    % Get knot velocities for iith dimension
    var{ii+1+n} = derivVals{2}(ii, :).';
end

PropertyNames = [{'Times [s]'}, ...
                 arrayfun(@(n) sprintf('q_%d [rad]', n), 1:n, 'UniformOutput', false), ...
                 arrayfun(@(n) sprintf('dq_%d [rad.s^-1]', n), 1:n, 'UniformOutput', false) ...
                 ];
% Create table
t = table(var{:}, 'VariableNames', PropertyNames);
% Write table to file
writetable(t, filename, 'WriteRowNames', false);

end