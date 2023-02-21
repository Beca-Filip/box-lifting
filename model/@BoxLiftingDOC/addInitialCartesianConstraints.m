function obj = addInitialCartesianConstraints(obj)

% Constraint:
obj.initialCartesianConstraints = obj.p_wri(:, 1) - obj.casadiLiftingEnvironment.WristInitialPosition;

% Set opti object to be subject to
obj.opti.subject_to(obj.initialCartesianConstraints == 0);
end