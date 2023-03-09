function w = matricizeWeightsForIOC(obj, w0)

% Out size 1
out_sz_1 = length(obj.doc.costFunctionVector);

% Reshape
w = reshape(w0, out_sz_1, []).';
end