function [boundaryConditions, opti] = getDefaultCasadiOptiBoundaryConditions(opti, dimension, degree, knotNumber)
%GETDEFAULTCASADIOPTIBOUNDARYCONDITIONS returns casadi opti parametrized boundary conditions on all
%derivatives up to order (degree-1)/2, at the first and last knot.
    % Initialize
    boundaryConditions = [];
    % Number of boundary conditions
    nbc = (degree-1);
    % For each pair of boundary conditions
    for bndCndPair = 1 : (nbc)/2
        % SplineBoundaryCondition constructor parameters
        Dimension = dimension;
        DerivativeOrder = bndCndPair;
        
        % Value at first point
        ValueFirst = opti.parameter(Dimension, 1);
        
        % The first knot of the pair upon which the condition will be placed
        KnotOfCondition = 1;
        
        % Construct new condition
        boundaryConditions = [boundaryConditions;
        SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, ValueFirst);
        ];
    
        % The second knot of the pair upon which the condition will be placed
        KnotOfCondition = knotNumber;
        
        % Value at last point
        ValueLast = opti.parameter(Dimension, 1);
        % Construct new condition
        boundaryConditions = [boundaryConditions;
        SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, ValueLast);
        ];
    end
        
        
end