function list = iterate_ndim(d, n, sz, index_arr, list)

% If d surpassed the last dimension
if d > n
    
    % Update the list and return
    list(end+1, :) = index_arr;
    return
end

% Go through indices of the d-th dimention
for ii = 1 : sz(d)
    
    % Set the d-th element of the index array
    index_arr(d) = ii;
    
    % Pass function further
    list = iterate_ndim(d+1, n, sz, index_arr, list);
    
end

end