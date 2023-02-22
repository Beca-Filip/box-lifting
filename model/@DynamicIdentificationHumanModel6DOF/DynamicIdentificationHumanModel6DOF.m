classdef DynamicIdentificationHumanModel6DOF
    %DYNAMICIDENTIFICATIONHUMANMODEL6DOF performs a dynamic identification
    %procedure for HumanModel6DOF, using casadi.
    
    properties
        
        % Number of samples of the IK
        nbSamples(1, 1)
        
        % Optimization object
        opti(1, 1)              casadi.Opti
        
        % Model object which is going to hold most of the optimization vars
        casadiHumanModel(1, 1)  HumanModel6DOF
        %%% Optimization Variables
        % Link center of mass positions in local link frames, units [m]
        CoM(2,6)
        % Mass scalars of the individual links, units [kg]
        M(1,6)
        % Moment of inertia scalars about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        Izz(1,6)
        % FEET CoM position in local link frame, units [m]
        CoMFOOT(2, 1)
        % HEAD CoM position in local link frame, units [m]
        CoMHAND(2, 1)
        % HAND CoM position in local link frame, units [m]
        CoMHEAD(2, 1)
        % FEET mass [kg]
        MFOOT(1, 1)
        % HAND mass [kg]
        MHAND(1, 1)
        % HEAD mass [kg]
        MHEAD(1, 1)
        % FEET moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        IzzFOOT(1, 1)
        % HAND moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        IzzHAND(1, 1)
        % HEAD moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        IzzHEAD(1, 1)
        
        % Vector from markers frame to force plate frame expressed in markers frame
        markers_r_markers_fp(3, 1)
        
        
        %%% Optimization Parameters
        % Reference human object: Contains all parameters of the human
        % model which are not variables, and contains reference values for
        % the variables
        casadiHumanModelRef(1, 1) HumanModel6DOF
        % Reference vector from markers frame to force plate frame expressed in markers frame       
        markers_r_markers_fp_ref(3, 1)
        % Vectore from markers frame to base frame expressed in markers frame
        markers_r_markers_base(3, 1)
        % Rotation matrix from forceplate frame to markers frame
        markers_R_fp(3, 3)
        % Rotation matrix from markers frame to base frame
        markers_R_base(3, 3)
        % Joint angles
        q(6, :)
        % Joint angular velocities
        dq(6, :)
        % Joint angular accelerations
        ddq(6, :)
        % Raw forceplate data object
        Forceplate
        % Forceplate data transported to base frame object
        TransportedForceplate
        % External forces
        fFOOT(6, :)
        fHAND(6, :)
        
        % Parameter deviation from ref
        refDeviation(1, 1)
        % Total mass deviation from ref
        totalMassDeviationFactor(1, 1)
        % Possible deviation of the vector from the markers frame to the forceplate frame in [m]
        maxDeviation_markers_r_markers_fp(3, 1);
        
        % Optimization functions        
        % Cost function object
        costFunction(1, 1)
        % Total mass constraint object
        totalMassConstraint(:, 1)
        % Mass limit constraints objet
        massLimitConstraints(:, 1)   
        % Inertia limit constraints
        inertiaLimitConstraints(:, 1) 
        % CoM limit constraints
        centerOfMassLimitConstraints(:, 1)
        % Torque constraints
        jointTorqueConstraints(:, 1)
        % Displacement of forceplate constraints
        displacementOfForceplateConstraints(:, 1)
        
        % Important subfunctions
        % Predicted joint torques
        tau(6, :)
        % Predicted ground reaction forces in base frame
        base_grf_model(6, :)
        % Predicted ground reaction forces in markers frame
        markers_grf_model(6, :)
        % Predicted cop position in markers frame
        markers_cop_model(1, :)
        % Forceplate ground reaction forces in forceplate frame
        fp_grf_fp(6, :)
        % Forceplate ground reaction forces in markers frame
        markers_grf_fp(6, :)
        % Forceplate cop position in markers frame
        markers_cop_fp(1, :)        
        % Residual objects
        res_fx
        res_fy
        res_mz
        res_cop
        cf_fx
        cf_fy
        cf_mz
        cf_cop
        % Part of the cost function containing all residuals
        residual_cost_function
        % Part of the cost function representing the normalization on the 
        % variation of wrenches from sample to sample
        wrench_variation_normalization_term
    end
    
    methods
        function obj = DynamicIdentificationHumanModel6DOF(nbSamples)
            %obj = DynamicIdentificationHumanModel6DOF(nbSamples)
            
            if nargin < 1
                nbSamples = 1;
            end
            
            % Store number of samples
            obj.nbSamples = nbSamples;
            
            % Allocate opti object
            obj.opti = casadi.Opti();
            
            % Allocate opti variables
            obj.CoM = obj.opti.variable(2, 6);
            obj.M = obj.opti.variable(1, 6);
            obj.Izz = obj.opti.variable(1, 6);
            obj.MFOOT = obj.opti.variable(1, 1);
            obj.MHAND = obj.opti.variable(1, 1);
            obj.MHEAD = obj.opti.variable(1, 1);
            obj.IzzFOOT = obj.opti.variable(1, 1);
            obj.IzzHAND = obj.opti.variable(1, 1);
            obj.IzzHEAD = obj.opti.variable(1, 1);
            obj.CoMFOOT = obj.opti.variable(2, 1);
            obj.CoMHAND = obj.opti.variable(2, 1);
            obj.CoMHEAD = obj.opti.variable(2, 1);
            obj.markers_r_markers_fp = obj.opti.variable(3, 1);
                        
            % Use human model class for future calculations
            obj.casadiHumanModel = HumanModel6DOF();    %%% CAREFUL INITIALIZES ALL OTHER PARAMETERS TO DEFAULT VALUES
            % Set all variables
            obj.casadiHumanModel.CoM = obj.CoM;
            obj.casadiHumanModel.M = obj.M;
            obj.casadiHumanModel.Izz = obj.Izz;
            obj.casadiHumanModel.MFOOT = obj.MFOOT;
            obj.casadiHumanModel.MHAND = obj.MHAND;
            obj.casadiHumanModel.MHEAD = obj.MHEAD;
            obj.casadiHumanModel.IzzFOOT = obj.IzzFOOT;
            obj.casadiHumanModel.IzzHAND = obj.IzzHAND;
            obj.casadiHumanModel.IzzHEAD = obj.IzzHEAD;
            obj.casadiHumanModel.CoMFOOT = obj.CoMFOOT;
            obj.casadiHumanModel.CoMHAND = obj.CoMHAND;
            obj.casadiHumanModel.CoMHEAD = obj.CoMHEAD;
            
            % Allocate opti parameters
            % Spatial parameters
            obj.markers_R_fp = obj.opti.parameter(3, 3);
            obj.markers_R_base = obj.opti.parameter(3, 3);
            obj.markers_r_markers_base = obj.opti.parameter(3, 1);
            obj.markers_r_markers_fp_ref = obj.opti.parameter(3, 1);
            
            % Joint angle parameters
            obj.q = obj.opti.parameter(6, nbSamples);
            obj.dq = obj.opti.parameter(6, nbSamples);
            obj.ddq = obj.opti.parameter(6, nbSamples);
            
            % Reference forceplate data parameters
            obj.Forceplate.Forces = obj.opti.parameter(nbSamples, 3);
            obj.Forceplate.Moments = obj.opti.parameter(nbSamples, 3);
            obj.Forceplate.COP = obj.opti.parameter(nbSamples, 3);
            
            % parameter: Reference human model
            obj.casadiHumanModelRef = HumanModel6DOF();
            % use default parameter initializer
            [obj.casadiHumanModelRef, obj.opti] = obj.casadiHumanModelRef.defaultCasadiOptiInitialize(obj.opti);
            % Set all non-reference human model parameters which are not variables
            obj.casadiHumanModel.WEIGHT = obj.casadiHumanModelRef.WEIGHT;
            obj.casadiHumanModel.HEIGHT = obj.casadiHumanModelRef.HEIGHT;
            obj.casadiHumanModel.R = obj.casadiHumanModelRef.R;
            obj.casadiHumanModel.p = obj.casadiHumanModelRef.p;
            obj.casadiHumanModel.L = obj.casadiHumanModelRef.L;
            obj.casadiHumanModel.HeelPosition = obj.casadiHumanModelRef.HeelPosition;
            obj.casadiHumanModel.ToePosition = obj.casadiHumanModelRef.ToePosition;
            obj.casadiHumanModel.LowerJointLimits = obj.casadiHumanModelRef.LowerJointLimits;
            obj.casadiHumanModel.UpperJointLimits = obj.casadiHumanModelRef.UpperJointLimits;
            obj.casadiHumanModel.LowerTorqueLimits = obj.casadiHumanModelRef.LowerTorqueLimits;
            obj.casadiHumanModel.UpperTorqueLimits = obj.casadiHumanModelRef.UpperTorqueLimits;
            obj.casadiHumanModel.LengthToRadiiFactor = obj.casadiHumanModelRef.LengthToRadiiFactor;
            
            % Define external forces
            obj.fFOOT = zeros(6, nbSamples);
            obj.fHAND = zeros(6, nbSamples);
            
            % Perform the inverse dynamics to get predicted ground reaction
            % forces
            [obj.tau, obj.base_grf_model] = obj.casadiHumanModel.inverseDynamicModel(obj.q, obj.dq, obj.ddq, obj.fFOOT, obj.fHAND);
            
            % Extract ground reaction forces from the forceplate
            obj.fp_grf_fp = [obj.Forceplate.Forces.'; obj.Forceplate.Moments.'];

            % Transport ground reaction forces from the base frame to the markers frame
            obj.markers_grf_model = WrenchesRotateTranslate(obj.markers_R_base, obj.markers_r_markers_base, obj.base_grf_model);

            % Transport ground reaction forces from the forceplate frame to the markers frame
            obj.markers_grf_fp = WrenchesRotateTranslate(obj.markers_R_fp, obj.markers_r_markers_fp, obj.fp_grf_fp);

            % Calculate the predicted COP in the markers frame
            obj.markers_cop_model = obj.markers_grf_model(6, :) ./ obj.markers_grf_model(2, :);
            
            % Calculate the forceplate COP in the markers frame
            obj.markers_cop_fp = obj.markers_grf_fp(6, :) ./ obj.markers_grf_fp(2, :);
            
            % Calculate the residuals
            obj.res_fx = obj.markers_grf_model(1, :).' - obj.markers_grf_fp(1, :).';
            obj.res_fy = obj.markers_grf_model(2, :).' - obj.markers_grf_fp(2, :).';
            obj.res_mz = obj.markers_grf_model(6, :).' - obj.markers_grf_fp(6, :).';
            obj.res_cop = obj.markers_cop_model.' - obj.markers_cop_fp.';
            
            % Calculate the cost functions
            obj.cf_fx = sum(sum(obj.res_fx.^2) ./ nbSamples);
            obj.cf_fy = sum(sum(obj.res_fy.^2) ./ nbSamples);
            obj.cf_mz = sum(sum(obj.res_mz.^2) ./ nbSamples);
            obj.cf_cop = sum(sum(obj.res_cop.^2) ./ nbSamples);
            
            obj.wrench_variation_normalization_term = sum((diff(obj.res_fx).^2 + diff(obj.res_fy).^2 + 100 * diff(obj.res_mz).^2) ./ (3 * nbSamples)); % "Variation Normalization"
            obj.residual_cost_function  = obj.cf_fx + obj.cf_fy + 100*obj.cf_mz;
            
            obj.costFunction = obj.residual_cost_function + 100 * obj.wrench_variation_normalization_term;
            
            % Add to opti object
            obj.opti.minimize(obj.costFunction);
            
            % Calculate the constraints
            % Define the reference deviation for parameters
            obj.refDeviation = 0.4;
            % Define the total mass deviation
            obj.totalMassDeviationFactor = 0.04;
            % Define the base to forceplate frame possible deviation in [m]
            obj.maxDeviation_markers_r_markers_fp = [0.05; 0.3; 0.05];
            
            % Define the total mass constraint
            obj.totalMassConstraint = [...
            -(sum(obj.M)+obj.MFOOT+obj.MHAND+obj.MHEAD) + (1 - obj.totalMassDeviationFactor) * obj.casadiHumanModelRef.WEIGHT;
             (sum(obj.M)+obj.MFOOT+obj.MHAND+obj.MHEAD) - (1 + obj.totalMassDeviationFactor) * obj.casadiHumanModelRef.WEIGHT;
            ];
            % Mass limit constraints objet
            obj.massLimitConstraints = [...
            -obj.M.' + (1 - obj.refDeviation) * obj.casadiHumanModelRef.M.';
             obj.M.' - (1 + obj.refDeviation) * obj.casadiHumanModelRef.M.';
            -obj.MFOOT + (1 - obj.refDeviation) * obj.casadiHumanModelRef.MFOOT;
             obj.MFOOT - (1 + obj.refDeviation) * obj.casadiHumanModelRef.MFOOT;
            -obj.MHAND + (1 - obj.refDeviation) * obj.casadiHumanModelRef.MHAND;
             obj.MHAND - (1 + obj.refDeviation) * obj.casadiHumanModelRef.MHAND;
            -obj.MHEAD + (1 - obj.refDeviation) * obj.casadiHumanModelRef.MHEAD;
             obj.MHEAD - (1 + obj.refDeviation) * obj.casadiHumanModelRef.MHEAD;
            ];
%             obj.massLimitConstraints = [...
%             -obj.M.' - obj.casadiHumanModelRef.M.';
%              obj.MFOOT - obj.casadiHumanModelRef.MFOOT;
%              obj.MHAND - obj.casadiHumanModelRef.MHAND;
%              obj.MHEAD - obj.casadiHumanModelRef.MHEAD;
%             ];
            % Inertia limit constraints
            obj.inertiaLimitConstraints = [...
            -obj.Izz.' + (1 - obj.refDeviation) * obj.casadiHumanModelRef.Izz.';
             obj.Izz.' - (1 + obj.refDeviation) * obj.casadiHumanModelRef.Izz.';
            -obj.IzzFOOT + (1 - obj.refDeviation) * obj.casadiHumanModelRef.IzzFOOT;
             obj.IzzFOOT - (1 + obj.refDeviation) * obj.casadiHumanModelRef.IzzFOOT;
            -obj.IzzHAND + (1 - obj.refDeviation) * obj.casadiHumanModelRef.IzzHAND;
             obj.IzzHAND - (1 + obj.refDeviation) * obj.casadiHumanModelRef.IzzHAND;
            -obj.IzzHEAD + (1 - obj.refDeviation) * obj.casadiHumanModelRef.IzzHEAD;
             obj.IzzHEAD - (1 + obj.refDeviation) * obj.casadiHumanModelRef.IzzHEAD;
            ];
%             obj.inertiaLimitConstraints = [...
%              obj.Izz.' - obj.casadiHumanModelRef.Izz.';
%              obj.IzzFOOT - obj.casadiHumanModelRef.IzzFOOT;
%              obj.IzzHAND - obj.casadiHumanModelRef.IzzHAND;
%              obj.IzzHEAD - obj.casadiHumanModelRef.IzzHEAD;
%             ];
            % CoM limit constraints
            %%% CAREFUL: HERE WE ALLOW THE ESTIMATION TO VARY W.R.T.
            %%% SEGMENT LENGTH
            obj.centerOfMassLimitConstraints = [...
            -obj.CoM(:) + (obj.casadiHumanModelRef.CoM(:) - obj.refDeviation * repmat(obj.casadiHumanModelRef.L(:), [2, 1]));
             obj.CoM(:) - (obj.casadiHumanModelRef.CoM(:) + obj.refDeviation * repmat(obj.casadiHumanModelRef.L(:), [2, 1]));
            -obj.CoMFOOT(:) + (obj.casadiHumanModelRef.CoMFOOT(:) - obj.refDeviation * repmat(obj.casadiHumanModelRef.LFOOT(:), [2, 1]));
             obj.CoMFOOT(:) - (obj.casadiHumanModelRef.CoMFOOT(:) + obj.refDeviation * repmat(obj.casadiHumanModelRef.LFOOT(:), [2, 1]));
            -obj.CoMHAND(:) + (obj.casadiHumanModelRef.CoMHAND(:) - obj.refDeviation * repmat(obj.casadiHumanModelRef.LHAND(:), [2, 1]));
             obj.CoMHAND(:) - (obj.casadiHumanModelRef.CoMHAND(:) + obj.refDeviation * repmat(obj.casadiHumanModelRef.LHAND(:), [2, 1]));
            -obj.CoMHEAD(:) + (obj.casadiHumanModelRef.CoMHEAD(:) - obj.refDeviation * repmat(obj.casadiHumanModelRef.LHEAD(:), [2, 1]));
             obj.CoMHEAD(:) - (obj.casadiHumanModelRef.CoMHEAD(:) + obj.refDeviation * repmat(obj.casadiHumanModelRef.LHEAD(:), [2, 1]));
            ];
%             obj.centerOfMassLimitConstraints = [...
%              obj.CoM(:) - obj.casadiHumanModelRef.CoM(:);
%              obj.CoMFOOT(:) - obj.casadiHumanModelRef.CoMFOOT(:);
%              obj.CoMHAND(:) - obj.casadiHumanModelRef.CoMHAND(:);
%              obj.CoMHEAD(:) - obj.casadiHumanModelRef.CoMHEAD(:);
%             ];
            % Torque constraints
            obj.jointTorqueConstraints = [...
            -obj.tau(:) + repmat(obj.casadiHumanModelRef.LowerTorqueLimits, [nbSamples, 1]);
             obj.tau(:) - repmat(obj.casadiHumanModelRef.UpperTorqueLimits, [nbSamples, 1]);
            ];
            % Displacement of model and forceplate constraints
            obj.displacementOfForceplateConstraints = [...
            -obj.markers_r_markers_fp + (obj.markers_r_markers_fp_ref - obj.maxDeviation_markers_r_markers_fp);
             obj.markers_r_markers_fp - (obj.markers_r_markers_fp_ref + obj.maxDeviation_markers_r_markers_fp);
            ];
        
            % Add to opti object
%             obj.opti.subject_to(obj.totalMassConstraint == 0);
%             obj.opti.subject_to(obj.massLimitConstraints == 0);
%             obj.opti.subject_to(obj.inertiaLimitConstraints == 0);
%             obj.opti.subject_to(obj.centerOfMassLimitConstraints == 0);
            obj.opti.subject_to(obj.totalMassConstraint <= 0);
            obj.opti.subject_to(obj.massLimitConstraints <= 0);
            obj.opti.subject_to(obj.inertiaLimitConstraints <= 0);
            obj.opti.subject_to(obj.centerOfMassLimitConstraints <= 0);
            obj.opti.subject_to(obj.jointTorqueConstraints <= 0);
            obj.opti.subject_to(obj.displacementOfForceplateConstraints <= 0);
%             obj.opti.subject_to(obj.displacementOfForceplateConstraints == 0);
        end
        
        function obj = instantiateParameters(obj, Forceplate, q, dq, ddq, humanModelRef, markers_R_base, markers_R_fp, markers_r_markers_base, markers_r_markers_fp)
            
            % Set Forceplate parameters
            obj.opti.set_value(obj.Forceplate.Forces, Forceplate.Forces);
            obj.opti.set_value(obj.Forceplate.Moments, Forceplate.Moments);
            obj.opti.set_value(obj.Forceplate.COP, Forceplate.COP);
            % Set rotation matrix from forceplate frame to base frame
            obj.opti.set_value(obj.markers_r_markers_base, markers_r_markers_base);
            obj.opti.set_value(obj.markers_r_markers_fp_ref, markers_r_markers_fp);
            obj.opti.set_value(obj.markers_R_base, markers_R_base);
            obj.opti.set_value(obj.markers_R_fp, markers_R_fp);
            % Set joint angles, velocities and accelerations
            obj.opti.set_value(obj.q, q);
            obj.opti.set_value(obj.dq, dq);
            obj.opti.set_value(obj.ddq, ddq);
            % Set human model reference values
            [obj.casadiHumanModelRef, obj.opti] = obj.casadiHumanModelRef.casadiInstantiateOptiParameters(humanModelRef, obj.opti);
            
            % Set initial values to the same as the reference values
            obj.opti.set_initial(obj.CoM, humanModelRef.CoM);
            obj.opti.set_initial(obj.M, humanModelRef.M);
            obj.opti.set_initial(obj.Izz, humanModelRef.Izz);
            obj.opti.set_initial(obj.MFOOT, humanModelRef.MFOOT);
            obj.opti.set_initial(obj.MHAND, humanModelRef.MHAND);
            obj.opti.set_initial(obj.MHEAD, humanModelRef.MHEAD);
            obj.opti.set_initial(obj.IzzFOOT, humanModelRef.IzzFOOT);
            obj.opti.set_initial(obj.IzzHAND, humanModelRef.IzzHAND);
            obj.opti.set_initial(obj.IzzHEAD, humanModelRef.IzzHEAD);
            obj.opti.set_initial(obj.CoMFOOT, humanModelRef.CoMFOOT);
            obj.opti.set_initial(obj.CoMHAND, humanModelRef.CoMHAND);
            obj.opti.set_initial(obj.CoMHEAD, humanModelRef.CoMHEAD);
            % Set model base to force plate vector
            obj.opti.set_initial(obj.markers_r_markers_fp, markers_r_markers_fp);
        end
        
        function residual_norms = computeResidualNorms(obj, solutionObj)
            residual_norms = [];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_fx).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_fy).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_mz).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_cop).^2, 2))];
        end
        
        function base_grf_model = computeBaseGRFModel(obj, solutionObj)
            base_grf_model = solutionObj.value(obj.base_grf_model);
        end
        
        function [markers_grf_model, markers_grf_fp] = computeMarkersGRFModelAndForceplate(obj, solutionObj)
            markers_grf_model = solutionObj.value(obj.markers_grf_model);
            markers_grf_fp = solutionObj.value(obj.markers_grf_fp);
        end
        
        function markers_r_markers_fp = compute_markers_r_markers_fp(obj, solutionObj)
            markers_r_markers_fp = solutionObj.value(obj.markers_r_markers_fp);
        end
        
        function numericHumanModel = computeNumericalModel(obj, solutionObj)
            %% Extract all parameters of a human model
            % Initialize it
            numericHumanModel = HumanModel6DOF();
            
            %% Parameters that are inside the casadiHumanModelRef object
            numericHumanModel.HEIGHT = solutionObj.value(obj.casadiHumanModelRef.HEIGHT);
            
            numericHumanModel.R = solutionObj.value(obj.casadiHumanModelRef.R);
            numericHumanModel.p = solutionObj.value(obj.casadiHumanModelRef.p);
            
            numericHumanModel.HeelPosition = solutionObj.value(obj.casadiHumanModelRef.HeelPosition);
            numericHumanModel.ToePosition = solutionObj.value(obj.casadiHumanModelRef.ToePosition);
            
            numericHumanModel.L = solutionObj.value(obj.casadiHumanModelRef.L);
            
            numericHumanModel.LFOOT = solutionObj.value(obj.casadiHumanModelRef.LFOOT);
            numericHumanModel.LHAND = solutionObj.value(obj.casadiHumanModelRef.LHAND);
            numericHumanModel.LHEAD = solutionObj.value(obj.casadiHumanModelRef.LHEAD);
            
            numericHumanModel.LowerJointLimits = solutionObj.value(obj.casadiHumanModelRef.LowerJointLimits);
            numericHumanModel.UpperJointLimits = solutionObj.value(obj.casadiHumanModelRef.UpperJointLimits);
            numericHumanModel.LowerTorqueLimits = solutionObj.value(obj.casadiHumanModelRef.LowerTorqueLimits);
            numericHumanModel.UpperTorqueLimits = solutionObj.value(obj.casadiHumanModelRef.UpperTorqueLimits);
            
            numericHumanModel.LengthToRadiiFactor = solutionObj.value(obj.casadiHumanModelRef.LengthToRadiiFactor);
            %% Parameters that are variables of the optimization            
            numericHumanModel.WEIGHT = solutionObj.value((sum(obj.M)+obj.MFOOT+obj.MHAND+obj.MHEAD));
            
            numericHumanModel.CoM = solutionObj.value(obj.CoM);
            numericHumanModel.M = solutionObj.value(obj.M);
            numericHumanModel.Izz = solutionObj.value(obj.Izz);
            
            numericHumanModel.CoMFOOT = solutionObj.value(obj.CoMFOOT);
            numericHumanModel.CoMHAND = solutionObj.value(obj.CoMHAND);
            numericHumanModel.CoMHEAD = solutionObj.value(obj.CoMHEAD);
            numericHumanModel.MFOOT = solutionObj.value(obj.MFOOT);
            numericHumanModel.MHAND = solutionObj.value(obj.MHAND);
            numericHumanModel.MHEAD = solutionObj.value(obj.MHEAD);
            numericHumanModel.IzzFOOT = solutionObj.value(obj.IzzFOOT);
            numericHumanModel.IzzHAND = solutionObj.value(obj.IzzHAND);
            numericHumanModel.IzzHEAD = solutionObj.value(obj.IzzHEAD);            
        end
    end
end