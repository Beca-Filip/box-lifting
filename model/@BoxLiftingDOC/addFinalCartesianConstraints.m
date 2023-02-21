function obj = addFinalCartesianConstraints(obj)

% Constraint:
obj.finalCartesianConstraints = obj.p_wri(:, end) - obj.casadiLiftingEnvironment.WristFinalPosition;

% Set opti object to be subject to
obj.opti.subject_to(obj.finalCartesianConstraints == 0);
end