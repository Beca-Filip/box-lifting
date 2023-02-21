classdef SplineTrajectory
    %SPLINETRAJECTORY creates a spline trajectory of arbitrary dimension.
    %It approximates a function using a piece-wise polynomial representation,
    %stores the coefficients of said polynomials, and uses them to compute
    %intermediate function values.
    
    properties (SetAccess = private)
        degree(1, 1)        % Degree of the spline, unitless
        dimension(1, 1)     % Dimension of the trajectory, unitless
        duration(1, 1)      % Duration of the trajectory, units [s]
        
        knotNumber(1, 1)    % Number of knots
        knotTimes(1, :)     % Times at which the knots occur, units [s]
        knotValues          % Values that occur at the knots, units of function output
        
        boundaryConditions(:, 1) SplineBoundaryCondition % Values that occur at the knots, units of function output
        
        coefficients    % Coefficients of the spline
        
        currentEvaluatedTimes(1, :)                     % Times at which the spline has most recently been evaluated, units [s]
        currentEvaluatedValuesAndDerivatives            % Cell array of values and derivative values
                                                        % that occur at the times at which the spline 
                                                        % has most recently been evaluated, of function output
        currentEvaluatedDerivatives(1, 1)               % Scalar that indicates which derivatives have been evaluated
    end
        
    methods
        function obj = SplineTrajectory(knotTimes, knotValues, degree, boundaryConditions)
            %SPLINETRAJECTORY Construct an instance of the spline trajectory
            %class.
            %   obj = SPLINETRAJECTORY(knotTimes, knotValues, degree, boundaryConditions)
            %   Takes in the knotTimes, knotValues, degree, and
            %   boundaryConditions.
                        
            % Get number of knots
            knotNumber = numel(knotTimes);
            
            % Treat number of inputs
            if nargin == 2
                % Set degree to 3
                degree = 3;                
                % Get dimension
                dimension = size(knotValues, 1);
                % Set default boundary conditions
                boundaryConditions = obj.getDefaultBoundaryConditions(dimension, degree, knotNumber);
            elseif nargin == 3
                % Get dimension
                dimension = size(knotValues, 1);
                % Set default boundary conditions
                boundaryConditions = obj.getDefaultBoundaryConditions(dimension, degree, knotNumber);
            elseif nargin == 4
            else
                error("knotTime and knotValues must be given at construction time.")
            end
            
            % Assign values from inputs
            obj.knotTimes = knotTimes;
            obj.knotValues = knotValues;
            obj.degree = degree;
            obj.boundaryConditions = boundaryConditions;
            
            % Get the implied values
            obj.knotNumber = numel(knotTimes);
            obj.dimension = size(obj.knotValues, 1);
            obj.duration = obj.knotTimes(end) - obj.knotTimes(1);
            
            % Compute the coefficients
            obj.coefficients = obj.computeCoefficients();
            
            % Current evaluation time and values are empty at object
            % creation
            obj.currentEvaluatedTimes = [];
            obj.currentEvaluatedValuesAndDerivatives = [];
            % -1 indicates no derivatives have been evaluated
            obj.currentEvaluatedDerivatives = -1;
        end
        
        % Computations of values at a given time vector
        V = computeValues(obj, computationTimes, derivativeOrder);
        obj = computeValuesAndStore(obj, computationTimes, derivativeOrder);
        % Casadi setters
        [obj, opti] = casadiInstantiateOptiParameters(obj, numericSplineTrajectory, opti);
    end
    
    methods (Static, Access = public)
        % Polynomial evaluation function
        p = computePolynomial(coefficients, computationTime, derivativeOrder);
       
        % Initializers
        [obj, opti] = defaultCasadiOptiInitialize(opti, dimension, degree, knotNumber, evaluationNumber);
                
        % Boundary conditions
        boundaryConditions = getDefaultBoundaryConditions(dimension, degree, knotNumber);
        
        % Equidistant spline knots
        [knotTimes, knotValues] = getEquidistantSplineKnots(q, t, numKnots);
    end
    
    methods (Access = private)
       % Computations of coefficients
       coefficients = computeCoefficients(obj);
    end
    
    methods (Static, Access = private)
       % Default boundary conditions
       [boundaryConditions, opti] = getDefaultCasadiOptiBoundaryConditions(opti, dimension, degree, knotNumber);
    end
end

