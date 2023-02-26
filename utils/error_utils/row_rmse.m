function e = row_rmse(a,b)
%ROW_RMSE calculates the root mean square error between the rows of two 
%signals of the same size.
%
%   e = ROW_RMSE(a,b)

e = sqrt(sum((a-b).^2, 2) ./ size(a, 2));

end