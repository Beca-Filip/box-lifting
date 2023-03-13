function obj = addFinalCartesianConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 5e-3;   % 0.005 m = 0.5 cm

% Constraint:
obj.finalCartesianConstraints = [...
obj.p_wri(:, end) - obj.casadiLiftingEnvironment.WristFinalPosition - constraintTol;
-obj.p_wri(:, end) + obj.casadiLiftingEnvironment.WristFinalPosition - constraintTol;
];

% Set opti object to be subject to
obj.opti.subject_to(obj.finalCartesianConstraints <= 0);
end