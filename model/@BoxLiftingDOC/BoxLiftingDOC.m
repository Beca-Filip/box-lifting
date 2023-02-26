classdef BoxLiftingDOC
    %BOXLIFTINGDOC creates an optimization model for box lifting with a
    %HumanModel6DOF.
        
    properties (Constant, GetAccess = private)
        mode1 = "constant_final_velocities"
        mode2 = "variable_final_velocities"
    end
    
    properties
        % Optimization object
        opti(1, 1)      casadi.Opti
        
        % Hyperparameters
        % Mode (with or without variable final constraints
        mode = "constant_final_velocities"
        % Dimension of spline
        splineDimension(1, 1) double = 6
        % Degree of spline
        splineDegree(1, 1) double = 5
        % Number of knots of spline
        splineKnotNumber(1, 1) double = 10
        % Number of evaluation points of spline
        splineEvaluationNumber(1, 1) double = 50
        
        
        % Optimization variables
        % Spline trajectory object
        casadiSplineTrajectory(1, 1)
        
        % Optimization parameters
        % Human model object
        casadiHumanModel(1, 1)              HumanModel6DOF
        % Lifting environment object
        casadiLiftingEnvironment(1, 1)      LiftingEnvironment
        
        % Cost function parameters
        omega(:, 1)
        
        % Optimization cost function set
        % Cost function vector:
        costFunctionVector(:, 1)
        % Cost function 1:
        sumSquaredJointVelocities(:, 1)
        % Cost function 2:
        sumSquaredJointAccelerations(:, 1)
        % Cost function 3:
        sumSquaredJointJerks(:, 1)
        % Cost function 4:
        sumSquaredWristVelocity(:, 1)
        % Cost function 5:
        sumSquaredWristAcceleration(:, 1)
        % Cost function 6:
        sumSquaredJointTorques(:, 1)
        % Cost function 7:
        sumSquaredJointPowers(:, 1)
        
        % Optimization compound cost function
        % Cost function:
        compoundCostFunction
        
        % Optimization constraints
        % Constraint function 1: jointLowerLimitConstraints
        jointLowerLimitConstraints(:, 1)
        % Constraint function 2: jointUpperLimitConstraints
        jointUpperLimitConstraints(:, 1)
        % Constraint function 3: initialJointConstraints
        initialJointConstraints(:, 1)
        % Constraint function 4: finalJointConstraints
        finalJointConstraints(:, 1)
        % Constraint function 5: initialCartesianConstraints
        initialCartesianConstraints(:, 1)
        % Constraint function 6: finalCartesianConstraints
        finalCartesianConstraints(:, 1)
        % Constraint function 7: collisionConstraints
        collisionConstraints(:, 1)
        % Constraint function 8: torqueLowerLimitConstraints
        torqueLowerLimitConstraints(:, 1)
        % Constraint function 9: torqueUpperLimitConstraints
        torqueUpperLimitConstraints(:, 1)
        % Constraint function 10: copLowerLimitConstraints
        copLowerLimitConstraints(:, 1)        
        % Constraint function 11: copUpperLimitConstraints
        copUpperLimitConstraints(:, 1)
        
        % Important quantities
        % Quantity 1: Joint angle trajectory
        q(6, :)
        % Quantity 2: Joint angle velocity
        dq(6, :)
        % Quantity 3: Joint angle acceleration
        ddq(6, :)
        % Quantity 4: Joint angle jerk
        dddq(6, :)
        % Quantity 5: External forces applied at the hand
        fHAND(6, :)
        % Quantity 6: Joint torques
        tau(6, :)
        % Quantity 7: Ground reaction forces
        f_grf(6, :)
        % Quantity 8: Center of pressure
        cop(1, :)
        % Quantity 9: Cartesian positions
        fkm
        % Quantity 10: Cartesian velocities
        v_fkm
        % Quantity 11: Cartesian accelerations
        a_fkm
        % Quantity 12: Wrist cartesian positions
        p_wri(2, :)
        % Quantity 13: Wrist cartesian velocities
        v_wri(2, :)
        % Quantity 12: Wrist cartesian accelerations
        a_wri(2, :)
    end
    
    methods
        function obj = BoxLiftingDOC(varargin)
            %BOXLIFTINGDOC Construct an instance of this class.

            %% Treat inputs
            % If input arguments are passed, assign them, else use the
            % default values
            if nargin > 0
                obj.splineDegree = varargin{1};
            end
            if nargin > 1
                obj.splineKnotNumber = varargin{2};
            end
            if nargin > 2
                obj.splineEvaluationNumber = varargin{3};
            end
            if nargin > 3
                if ~strcmp(varargin{4},obj.mode1) && ~strcmp(varargin{4}, obj.mode2)
                    error("undefined mode");
                end
                obj.mode = varargin{4};
            end
            
            %% Create default opti object
            % Create the opti object
            obj.opti = casadi.Opti();
            
            %% Create parameters
            % Create default casadi symbolic human model
            obj.casadiHumanModel = HumanModel6DOF();
            [obj.casadiHumanModel, obj.opti] = obj.casadiHumanModel.defaultCasadiOptiInitialize(obj.opti);
            
            % Create default casadi symbolic lifting environment model
            obj.casadiLiftingEnvironment = LiftingEnvironment("casadi", obj.opti);
            
            % Create default spline trajectory object
            if strcmp(obj.mode, obj.mode1)
                [obj.casadiSplineTrajectory, obj.opti] = ...
                SplineTrajectory.defaultCasadiOptiInitialize(obj.opti, obj.splineDimension, obj.splineDegree, obj.splineKnotNumber, obj.splineEvaluationNumber);
            elseif strcmp(obj.mode, obj.mode2)
                [obj.casadiSplineTrajectory, obj.opti] = ...
                SplineTrajectory.defaultCasadiOptiInitializeVarConditions(obj.opti, obj.splineDimension, obj.splineDegree, obj.splineKnotNumber, obj.splineEvaluationNumber);
            end
            %% Extract important quantities
            % Get joint trajectories
            obj.q = obj.casadiSplineTrajectory.currentEvaluatedValuesAndDerivatives{1};
            obj.dq = obj.casadiSplineTrajectory.currentEvaluatedValuesAndDerivatives{2};
            obj.ddq = obj.casadiSplineTrajectory.currentEvaluatedValuesAndDerivatives{3};
            obj.dddq = obj.casadiSplineTrajectory.currentEvaluatedValuesAndDerivatives{4};
            
            % Get external forces
            fFOOT = zeros(6, obj.splineEvaluationNumber);
            obj.fHAND = obj.casadiLiftingEnvironment.getExternalWrenches(obj.q, obj.casadiHumanModel.Gravity);
            
            % Get the joint torques and ground reaction forces
            [obj.tau, obj.f_grf] = obj.casadiHumanModel.inverseDynamicModel(obj.q, obj.dq, obj.ddq, fFOOT, obj.fHAND);
            
            % Get the center of pressure
            obj.cop = obj.f_grf(6, :) ./ obj.f_grf(2, :);
            
            % Get the cartesian positions
            obj.fkm = obj.casadiHumanModel.forwardKinematicModel(obj.q);
            % Get the cartesian velocities
            obj.v_fkm = obj.casadiHumanModel.forwardKinematicVelocityModel(obj.q, obj.dq);
            % Get the cartesian velocities
            obj.a_fkm = obj.casadiHumanModel.forwardKinematicAccelerationModel(obj.q, obj.dq, obj.ddq);
            
            % Extract wrist position, velocity, acceleration
            obj.p_wri = obj.fkm{contains([obj.casadiHumanModel.KPOI.Name], "Hands")}(1:2, :);
            obj.v_wri = obj.v_fkm{contains([obj.casadiHumanModel.KPOI.Name], "Hands")}(1:2, :);
            obj.a_wri = obj.a_fkm{contains([obj.casadiHumanModel.KPOI.Name], "Hands")}(1:2, :);
            
            % Extract sphere distances
            obj.casadiHumanModel
            %% Initialize empty problem cost
            % Cost function parameters
            obj.omega = [];
            % Optimization cost function set
            % Cost function vector:
            obj.costFunctionVector = [];
            % Cost function 1:
            obj.sumSquaredJointVelocities = [];
            % Cost function 2:
            obj.sumSquaredJointAccelerations = [];
            % Cost function 3:
            obj.sumSquaredJointJerks = [];
            % Cost function 4:
            obj.sumSquaredWristVelocity = [];
            % Cost function 5:
            obj.sumSquaredWristAcceleration = [];
            % Cost function 6:
            obj.sumSquaredJointTorques = [];
            % Cost function 7:
            obj.sumSquaredJointPowers = [];

            % Optimization compound cost function
            % Cost function:
            obj.compoundCostFunction = [];
            %% Initialize empty problem constraints
            % Constraint 1: Joint lower limit constraints
            obj.jointLowerLimitConstraints = [];
            % Constraint 2: Joint upper limit constraints
            obj.jointUpperLimitConstraints =  [];
            % Constraint function 3: initialJointConstraints
            obj.initialJointConstraints = [];
            % Constraint function 4: finalJointConstraints
            obj.finalJointConstraints = [];
            % Constraint function 5: initialCartesianConstraints
            obj.initialCartesianConstraints = [];
            % Constraint function 6: finalCartesianConstraints
            obj.finalCartesianConstraints = [];
            % Constraint function 7: collisionConstraints
            obj.collisionConstraints = [];
        end
        
        % Constraint adders
        obj = addJointLimitConstraints(obj);
        obj = addInitialJointConstraints(obj);
        obj = addFinalJointConstraints(obj);
        obj = addInitialCartesianConstraints(obj);
        obj = addFinalCartesianConstraints(obj);
        obj = addCollisionConstraints(obj);
        obj = addTorqueLimitConstraints(obj);
        obj = addCopLimitConstraints(obj);
        
        % Cost function adders        
        obj = addSumSquaredJointVelocitiesCost(obj);
        obj = addSumSquaredJointAccelerationsCost(obj);
        obj = addSumSquaredJointJerksCost(obj);
        obj = addSumSquaredWristVelocityCost(obj);
        obj = addSumSquaredWristAccelerationCost(obj);
        obj = addSumSquaredJointTorquesCost(obj);
        obj = addSumSquaredJointPowersCost(obj);
        
        % Cost function setter
        obj = setParametrizedCostFunction(obj);
        
        % Instantiator
        function obj = instantiateParameters(obj, numericHumanModel, numericLiftingEnvironment, numericSplineTrajectory)
            
            % Instantiate parameters through the human model and lifting
            % environment class interfaces
            [obj.casadiHumanModel, obj.opti] = obj.casadiHumanModel.casadiInstantiateOptiParameters(numericHumanModel, obj.opti);
            [obj.casadiLiftingEnvironment, obj.opti] = obj.casadiLiftingEnvironment.casadiInstantiateOptiParameters(numericLiftingEnvironment, obj.opti);
            
            % Instantiate variables and parameters through the spline trajectory class
            % interface
            if strcmp(obj.mode, obj.mode1)
                [obj.casadiSplineTrajectory, obj.opti] = obj.casadiSplineTrajectory.casadiInstantiateOptiParameters(numericSplineTrajectory, obj.opti);
            elseif strcmp(obj.mode, obj.mode2)
                [obj.casadiSplineTrajectory, obj.opti] = obj.casadiSplineTrajectory.casadiInstantiateOptiParametersVarConditions(numericSplineTrajectory, obj.opti);
            end
        end
    end
end

