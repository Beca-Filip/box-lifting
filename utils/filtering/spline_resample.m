function qr = spline_resample(q, N)
%SPLINE_RESAMPLE resamples a sequence q, using default third-order spline 
%interpolation inbetween the points.
%   qr = SPLINE_RESAMPLE(q, N) resamples the sequence q starting and ending
%   with the same value, using N points.

t = linspace(0, 1, size(q, 2));
tr = linspace(0, 1, N);

qr = spline(t, q, tr);
end