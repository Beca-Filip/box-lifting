function [Forceplate] = ForceplateRotateTranslate(R,p,Forceplate)
%FORCEPLATEROTATETRANSLATE first rotates and then translate the Forceplate 
%structure.

% reshape the position vector
p = reshape(p, [1, 3]);
skewP = [[0, -p(3), p(2)]; [p(3), 0, -p(1)]; [-p(2), p(1), 0]];

% Get the markerset names
forceplateFieldnames = fieldnames(Forceplate);

% For each markerset
for numFieldname = 1 : length(forceplateFieldnames)
    
    % Rotate the vectors
    Forceplate.(forceplateFieldnames{numFieldname}) = (R * Forceplate.(forceplateFieldnames{numFieldname}).').';

end

% Moments and CoP change witht ranslation
Forceplate.Moments = Forceplate.Moments + ((skewP) * Forceplate.Forces.').';
Forceplate.COP = Forceplate.COP + repmat(p, [size(Forceplate.COP, 1), 1]);
end

