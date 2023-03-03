function w = expandWeightsForDOC(obj, w0)

% Num samples
nSamples = size(obj.doc.omega_t, 2);

% Size of parameter 
[nCF, nT] = size(w0);

% Create a new parameter w
w = ones(nCF, nSamples);

% Division
divVec = round(linspace(1, nSamples, nT + 1));

% Assign weights
for tt = 1 : nT
    
    % Get current indices
    curr_indices = divVec(tt) : divVec(tt+1);
    % For times between
    w(:, curr_indices) = ...
        repmat(w0(:, tt), [1, length(curr_indices)]);
    
end

end