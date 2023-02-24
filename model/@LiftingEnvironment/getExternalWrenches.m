function f = getExternalWrenches(obj,q,Gravity)
%GETEXTERNALWRENCHES calculates the eternal wrenches applied at the hand
%given the joint angles vector.
%   f = getExternalWrenches(obj,q,Gravity)
%   Gravity = - 9.81 to get model action forces on the environment.

% Get the number of samples
nbSamples = size(q, 2);

% Get the wrist to box vector in the box frame (global)
r = obj.WristToBoxGripPointVector;

% Get the box mass
BoxMass = obj.BoxMass;

% Moments are constant
mm = [zeros(2, nbSamples); repmat(-r(1)*BoxMass*Gravity, [1, nbSamples])];

% Forces change depending on the orientation\
ff = [sin(-sum(q, 1)) .* BoxMass .* Gravity; 
     -cos(-sum(q, 1)) .* BoxMass .* Gravity;
     zeros(1, nbSamples);];
 
% Concatenate and return
f = [ff; mm];
end

