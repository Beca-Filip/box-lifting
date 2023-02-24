function boundaryConditions = fifthOrderBoundaryConditions(numKnots, dq1, ddq1, dqend, ddqend)
%FIFTHORDERBOUNDARYCONDITIONS returns boundary conditions for the 1st and
%2nd derivative, on the first and last knot, given the values for the
%derivative conditions.
    
    % Fixed dimension of spline
    Dimension = size(dq1, 1);
    
    % Initialize boundary condition array
    boundaryConditions = [];
    
    % First derivatives
    DerivativeOrder = 1;
    
    % Initial Velocity
    Value = dq1;
    KnotOfCondition = 1;    
    % Boundary conditions update
    boundaryConditions = [boundaryConditions;
    SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
    ];
    % Final Velocity
    Value = dqend;
    KnotOfCondition = numKnots;    
    % Boundary conditions update
    boundaryConditions = [boundaryConditions;
    SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
    ];

    % Second derivatives
    DerivativeOrder = 2;
    
    % Initial Acceleration
    Value = ddq1;
    KnotOfCondition = 1;    
    % Boundary conditions update
    boundaryConditions = [boundaryConditions;
    SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
    ];
    % Final Velocity
    Value = ddqend;
    KnotOfCondition = numKnots;    
    % Boundary conditions update
    boundaryConditions = [boundaryConditions;
    SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
    ];
end