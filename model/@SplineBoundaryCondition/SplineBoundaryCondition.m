classdef SplineBoundaryCondition
    %SPLINEBOUNDARYCONDITION is an object which contains info on boundary
    %conditions to be imposed on a spline trajectory.
    
    properties (SetAccess = private)
        % All properties
        
        % Dimension of the trajectory upon which the boundary condition is
        % placed
        Dimension(1, 1) {mustBeInteger}
        
        % The order of the derivative upon which the boundary condition is
        % placed
        DerivativeOrder(1, 1) {mustBeInteger}
        
        % The knot upon the which the condition is posed
        KnotOfCondition(1, 1)
        
        % The value of the condition posed
        Value(:, 1)
    end
    
    methods
        function obj = SplineBoundaryCondition(Dimension, DerivativeOrder,KnotOfCondition,Value)
            %SPLINEBOUNDARYCONDITION Construct an instance of this class
            %Must be given the DerivativeOrder (>= 1), KnotOfCondition
            %(1 <= KnotOfCondition <= knotNumber), and Value (==
            %dimension).
            
            if (nargin == 0)
                Dimension = 1;
                DerivativeOrder = 1;
                KnotOfCondition = 1;
                Value = 0;
            end
            obj.Dimension = Dimension;
            obj.DerivativeOrder = DerivativeOrder;
            obj.KnotOfCondition = KnotOfCondition;
            obj.Value = Value;
        end
    end
end

