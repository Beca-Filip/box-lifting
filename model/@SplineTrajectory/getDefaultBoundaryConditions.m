function boundaryConditions = getDefaultBoundaryConditions(dimension, degree, knotNumber)
%GETDEFAULTBOUNDARYCONDITIONS returns zero boundary conditions on all
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
        Value = zeros(Dimension, 1);
        
        % The first knot of the pair upon which the condition will be placed
        KnotOfCondition = 1;
        
        % Construct new condition
        boundaryConditions = [boundaryConditions;
        SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
        ];
    
        % The second knot of the pair upon which the condition will be placed
        KnotOfCondition = knotNumber;
        
        % Construct new condition
        boundaryConditions = [boundaryConditions;
        SplineBoundaryCondition(Dimension, DerivativeOrder, KnotOfCondition, Value);
        ];
    end
        
        
end

% % Default boundary conditions
% function boundaryConditions = getDefaultBoundaryConditions(dimension, degree, knotTimes)
%         % Order of derivative boundary conditions
%         nbc = (degree-1)/2;
%         % Create boundary conditions which sets all derivatives up
%         % to order nbc, at the 1st and last knot to zero, for all
%         % dimensions
%         % Preallocate
% %                 boundaryConditions = zeros(2*nbc, 2 + dimension, class(knotTimes));
%         boundaryConditions = zeros(2*nbc, 2 + dimension);
%         % Set derivative orders
%         boundaryConditions(1:nbc, 1) = 1:nbc;
%         boundaryConditions(nbc+1:end, 1) = 1:nbc;
%         % Set condition times
%         boundaryConditions(1:nbc, 2) = knotTimes(1);
%         boundaryConditions(nbc+1:end, 2) = knotTimes(end);
%         % Values are already set to 0
% end