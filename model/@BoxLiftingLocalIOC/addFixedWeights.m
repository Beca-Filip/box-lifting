function w = addFixedWeights(obj, w0)

% Adds the fixed weights to the beginning of the array
w = vertcat(obj.fixedWeights, w0);

end