function list = list_ndim(n, sz)

% Initialize dimension
d = 1;
% Initialize index array
index_arr = zeros(1, n);
% Initialize list
list = [];

% Call the iterations
list = iterate_ndim(d, n, sz, index_arr, list);
end