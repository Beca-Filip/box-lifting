classdef InverseKinematicsHumanModel6DOF
    %INVERSEKINEMATICSHUMANMODEL6DOF performs an inverse kinematics
    %procedure for HumanModel6DOF, using casadi.
    
    properties
        
        % Number of samples of the IK
        nbSamples(1, 1)
        
        % Searching for lengths flag
        modeFlag
        
        % Optimization object
        opti(1, 1)              casadi.Opti
        
        % Optimization Variables
        % Model object
        casadiHumanModel(1, 1)  HumanModel6DOF
        % Segment lengths object
        L(1, 6)     
        % Joint angles object
        q(6, :)
        
        % Optimization Parameters
        % Joint limits
        LowerJointLimits(6, 1)  
        UpperJointLimits(6, 1)  
        % Reference segment lengths
        LowerLengthLimits(1, 6) 
        UpperLengthLimits(1, 6) 
        % Markers object
        Markers
        
        % Optimization functions        
        % Cost function object
        costFunction(1, 1)      
        % Lengths limit constraints objet
        lengthLimitConstraints(:, 1)   
        % Joint limit constraints object
        jointLimitConstraints(:, 1)    
        
        % Important subfunctions
        % Residual objects
        res_rkne    
        res_rgtr    
        res_back    
        res_rsho    
        res_relb    
        res_rwri    
        cf_rkne    
        cf_rgtr    
        cf_back    
        cf_rsho    
        cf_relb    
        cf_rwri    
    end
    
    methods
        function obj = InverseKinematicsHumanModel6DOF(nbSamples,modeFlag)
            %obj = InverseKinematicsHumanModel6DOF(nbSamples,modeFlag)
            %nbSamples number of samples of the IK
            %modeFlag == 1 fixed lengths, search for joint angles
            %modeFlag == 2 fixed joint angles, search for lengths
            %modeFlag == 3 search for both simultaneously
            
            if nargin < 1
                nbSamples = 1;
            end
            if nargin < 2
                modeFlag = 1;
            end
            if modeFlag < 1 || modeFlag > 3
                error("modeFlag must be 1, 2, or 3");
            end
            
            % Store number of samples
            obj.nbSamples = nbSamples;
            % Store flag
            obj.modeFlag = modeFlag;
            
            % Allocate opti object
            obj.opti = casadi.Opti();
            
            % Allocate opti variables
            if modeFlag == 1
                % In default mode, lengths are parameters
                obj.L = obj.opti.parameter(1, 6);
                % Joint angles are variables
                obj.q = obj.opti.variable(6, nbSamples);
            elseif modeFlag == 2                
                % In length mode variables are lengths
                obj.L = obj.opti.variable(1, 6);
                % Joint angles are parameters
                obj.q = obj.opti.parameter(6, nbSamples);
            elseif modeFlag == 3
                % In both mode lengths are variables
                obj.L = obj.opti.variable(1, 6);
                % Joint angles are variables
                obj.q = obj.opti.variable(6, nbSamples);                
            end
                
            
            % Create human model with default kinematic points
            obj.casadiHumanModel = HumanModel6DOF();
            obj.casadiHumanModel.L = obj.L;
            obj.casadiHumanModel = obj.casadiHumanModel.setDefaultKinematicPointsOfInterest();
            
            % Create struct
            % Knee
            obj.Markers.BODY.RKNE = obj.opti.parameter(nbSamples, 2);
            % Hip (Great trochanter)
            obj.Markers.BODY.RGTR = obj.opti.parameter(nbSamples, 2);
            % Back
            obj.Markers.BODY.BACK = obj.opti.parameter(nbSamples, 2);
            % Shoulder
            obj.Markers.BODY.RSHO = obj.opti.parameter(nbSamples, 2);
            % Elbow
            obj.Markers.BODY.RELB = obj.opti.parameter(nbSamples, 2);
            % Wrist
            obj.Markers.BODY.RWRI = obj.opti.parameter(nbSamples, 2);
            
            % Create cost function
            % Calculate FKM
            fkm = obj.casadiHumanModel.forwardKinematicModel(obj.q);
            % Residuals
            obj.res_rkne = fkm{2}(1:2, :).' - obj.Markers.BODY.RKNE;
            obj.res_rgtr = fkm{3}(1:2, :).' - obj.Markers.BODY.RGTR;
            obj.res_back = fkm{4}(1:2, :).' - obj.Markers.BODY.BACK;
            obj.res_rsho = fkm{5}(1:2, :).' - obj.Markers.BODY.RSHO;
            obj.res_relb = fkm{6}(1:2, :).' - obj.Markers.BODY.RELB;
            obj.res_rwri = fkm{7}(1:2, :).' - obj.Markers.BODY.RWRI;
            % Cost function parts
            obj.cf_rkne = sum(sum(obj.res_rkne.^2) ./ (2*nbSamples));
            obj.cf_rgtr = sum(sum(obj.res_rgtr.^2) ./ (2*nbSamples));
            obj.cf_back = sum(sum(obj.res_back.^2) ./ (2*nbSamples));
            obj.cf_rsho = sum(sum(obj.res_rsho.^2) ./ (2*nbSamples));
            obj.cf_relb = sum(sum(obj.res_relb.^2) ./ (2*nbSamples));
            obj.cf_rwri = sum(sum(obj.res_rwri.^2) ./ (2*nbSamples));
            % Cost function
            obj.costFunction = obj.cf_rkne + obj.cf_rgtr + obj.cf_back + ...
                               obj.cf_rsho + obj.cf_relb + obj.cf_rwri;
                           
            % Add to opti object
            obj.opti.minimize(obj.costFunction);
            
            % Constraint functions
            if modeFlag == 1
                % In default mode, there are constraints on the joint
                % variables
                % Joint angle constraints
                obj.LowerJointLimits = obj.opti.parameter(6, 1);
                obj.UpperJointLimits = obj.opti.parameter(6, 1);
                obj.jointLimitConstraints = [-obj.q(:) + repmat(obj.LowerJointLimits, [nbSamples, 1]);
                                              obj.q(:) - repmat(obj.UpperJointLimits, [nbSamples, 1])];
                % No length constraints
                obj.LowerLengthLimits = zeros(1, 6);
                obj.UpperLengthLimits = zeros(1, 6);
                obj.lengthLimitConstraints = [];
                
                % Add to opti object
                obj.opti.subject_to(obj.jointLimitConstraints <= 0);
            elseif modeFlag == 2
                % In length search mode there are constraints on the
                % lengths
                % Length constraints
                obj.LowerLengthLimits = obj.opti.parameter(1, 6);
                obj.UpperLengthLimits = obj.opti.parameter(1, 6);
                obj.lengthLimitConstraints = [-obj.casadiHumanModel.L.' + obj.LowerLengthLimits.';
                                           obj.casadiHumanModel.L.' - obj.UpperLengthLimits.'];
                % No joint angle constraints
                obj.LowerJointLimits = zeros(6, 1);
                obj.UpperJointLimits = zeros(6, 1);
                obj.jointLimitConstraints = [];
                
                % Add to opti object
                obj.opti.subject_to(obj.lengthLimitConstraints <= 0);
            elseif modeFlag == 3
                % In complete mode, there are constraints on the joint
                % variables and on the length variables
                % Joint angle constraints
                obj.LowerJointLimits = obj.opti.parameter(6, 1);
                obj.UpperJointLimits = obj.opti.parameter(6, 1);
                obj.jointLimitConstraints = [-obj.q(:) + repmat(obj.LowerJointLimits, [nbSamples, 1]);
                                              obj.q(:) - repmat(obj.UpperJointLimits, [nbSamples, 1])];
                % No length constraints
                obj.LowerLengthLimits = obj.opti.parameter(1, 6);
                obj.UpperLengthLimits = obj.opti.parameter(1, 6);
                obj.lengthLimitConstraints = [-obj.casadiHumanModel.L.' + obj.LowerLengthLimits.';
                                           obj.casadiHumanModel.L.' - obj.UpperLengthLimits.'];
                
                % Add to opti object
                obj.opti.subject_to(obj.jointLimitConstraints <= 0);
                obj.opti.subject_to(obj.lengthLimitConstraints <= 0);                
            end
        end
        
        function obj = instantiateParameters(obj, Markers, q, L, varargin)
            %modeFlag == 1:
            %   obj = instantiateParameters(obj, Markers, q0, L, LowerJointLimits, UpperJointLimits)
            %modeFlag == 2:
            %   obj = instantiateParameters(obj, Markers, q, L0, LowerLengthLimits, UpperLengthLimits)
            %modeFlag == 3:
            %   obj = instantiateParameters(obj, Markers, q0, L0, LowerJointLimits, UpperJointLimits, LowerLengthLimits, UpperLengthLimits)

            if obj.modeFlag == 1
                % Check inputs
                if nargin ~= 6
                    error("modeFlag == 1: Call obj = instantiateParameters(obj, Markers, q0, L, LowerJointLimits, UpperJointLimits)");
                end
                
                % Get inputs and set them as parameters
                LowerJointLimits = varargin{1};
                UpperJointLimits = varargin{2};
                obj.opti.set_value(obj.LowerJointLimits, LowerJointLimits);
                obj.opti.set_value(obj.UpperJointLimits, UpperJointLimits);
                
                % In default mode, joint angles are variables and lengths
                % are parameters
                % Set initial value for joint variables
                obj.opti.set_initial(obj.q, q);
                % Set length parameter value
                obj.opti.set_value(obj.L, L);
            elseif obj.modeFlag == 2
                % Check inputs
                if nargin ~= 6
                    error("modeFlag == 2: Call instantiateParameters(obj, Markers, q, L0, LowerLengthLimits, UpperLengthLimits)");
                end
                
                % Get inputs and set them as parameters
                LowerLengthLimits = varargin{1};
                UpperLengthLimits = varargin{2};
                obj.opti.set_value(obj.LowerLengthLimits, LowerLengthLimits);
                obj.opti.set_value(obj.UpperLengthLimits, UpperLengthLimits);
                
                % In default mode, joint angles are variables and lengths
                % are parameters
                % Set joint angle parameter value
                obj.opti.set_value(obj.q, q);
                % Set length variable initial value
                obj.opti.set_initial(obj.L, L);
            elseif obj.modeFlag == 3
                % Check inputs
                if nargin ~= 8
                    error("modeFlag == 3: Call obj = instantiateParameters(obj, Markers, q0, L0, LowerJointLimits, UpperJointLimits, LowerLengthLimits, UpperLengthLimits)");
                end
                
                % Get inputs and set them as parameters
                LowerJointLimits = varargin{1};
                UpperJointLimits = varargin{2};
                LowerLengthLimits = varargin{3};
                UpperLengthLimits = varargin{4};
                obj.opti.set_value(obj.LowerJointLimits, LowerJointLimits);
                obj.opti.set_value(obj.UpperJointLimits, UpperJointLimits);
                obj.opti.set_value(obj.LowerLengthLimits, LowerLengthLimits);
                obj.opti.set_value(obj.UpperLengthLimits, UpperLengthLimits);
                
                % In default mode, joint angles are variables and lengths
                % are parameters
                % Set joint angle parameter value
                obj.opti.set_initial(obj.q, q);
                % Set length variable initial value
                obj.opti.set_initial(obj.L, L);                
            end
            
            % Set parameters unaffected by modeFlag
            % Instantiate markers
            markerNames = fieldnames(obj.Markers.BODY);
            
            % Set marker parameter values
            for numMarker = 1 : length(markerNames)
                obj.opti.set_value(obj.Markers.BODY.(markerNames{numMarker}), ...
                                       Markers.BODY.(markerNames{numMarker})(:, 1:2));
            end
        end
        
        function residual_norms = computeResidualNorms(obj, solutionObj)
            residual_norms = [];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_rkne).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_rgtr).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_back).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_rsho).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_relb).^2, 2))];
            residual_norms = [residual_norms, sqrt(sum(solutionObj.value(obj.res_rwri).^2, 2))];
        end
    end
end

