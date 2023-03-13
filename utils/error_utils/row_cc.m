function CC = row_cc(a,b)
%ROW_CC calculates the correlation coefficient between the rows of two 
%signals of the same size.
%
%  CC = ROW_CC(a,b)

CC = zeros(size(a, 1), 1);
for ii = 1 : size(a, 1)
    CC(ii) = corr2(a(ii, :), b(ii, :));
end
end