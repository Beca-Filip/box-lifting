function [f_grf_trans] = WrenchesTranslateRotate(p,R,f_grf)
%WRENCHESTRANSLATEROTATE first translates then rotates 6DoF wrenches.

% Reshape the position vector
p = reshape(p, [1, 3]);
skewP = [[0, -p(3), p(2)]; [p(3), 0, -p(1)]; [-p(2), p(1), 0]];

% Reassign
f_grf_trans = f_grf;
% Transport moment
f_grf_trans(4:6, :) = f_grf(4:6, :) + skewP * f_grf(1:3, :);
% Rotate both forces and moments
f_grf_trans(1:3, :) = R * f_grf_trans(1:3, :);
f_grf_trans(4:6, :) = R * f_grf_trans(4:6, :);
end

