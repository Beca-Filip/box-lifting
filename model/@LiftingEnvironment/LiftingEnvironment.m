classdef LiftingEnvironment
    %LIFTINGENVIRONMENT encapsulates all the information related to the
    %lifting task.
    
    properties
        % Box-related properties
        BoxWidth(1, 1)  % Width of the box, units [m]
        BoxHeight(1, 1) % Height of the box, units [m]
        BoxMass(1, 1)   % Mass of the box, units [kg]
        
        % Human-Box relative properties
        WristToBoxGripPointVector(2, 1) % 2D coordinates of the grip point
        
        % Table properties
        TableWidth(1, 1)                % Width of the table, units [m]
        TableHeight(1, 1)               % Height of the table, units [m]
        TableCenterCoordinates(2, 1)    % 2D coordinates of the center of the table
                                        % expressed in the HumanModel6DOF base frame, units [m]
        
        % Related to initial and final human posture
        WristInitialPosition(2, 1)      % 2D coordinates of the wrist initial position
                                        % expressed in the HumanModel6DOF base frame, units [m]
        WristFinalPosition(2, 1)        % 2D coordinates of the wrist final position
                                        % expressed in the HumanModel6DOF base frame, units [m]
        
        JointAnglesInitial(6, 1)    % 6D initial joint angles, units [rad]
        JointAnglesFinal(6, 1)      % 6D initial joint angles, units [rad]
    end
    
    methods
        function obj = LiftingEnvironment(initMethod, opti)
            %LIFTINGENVIRONMENT Construct an instance of this class
            %
            
            if nargin == 0
                obj = obj.defaultNumericInitialize();
                return
            end
            
            if strcmp(initMethod, "casadi") || strcmp(initMethod, "casadi.SX") || strcmp(initMethod, "casadi.MX")
                obj = obj.defaultCasadiOptiInitialize(opti);
                return
            end
            
        end
        
        % Print to file
        printToFile(obj, filename);
        
        % Default information getters
        [BoxWidth, BoxHeight, BoxMass] = defaultBoxParameters(obj);
        [TableWidth, TableHeight, TableCenterCoordinates] = defaultTableParameters(obj);
        [WristInitialPosition, WristFinalPosition] = defaultWristPositionParameters(obj);
        [WristToBoxGripPointVector] = defaultGripPointParameters(obj);
        
        % Information getters
        f = getExternalWrenches(obj, q, Gravity);
        collisionSphere = createBoxCollisionSphere(obj);
        collisionSphere = createTableCollisionSphere(obj);
        
        % Initializers
        obj = defaultNumericInitialize(obj);
        obj = defaultCasadiInitialize(obj);
        [obj, opti] = defaultCasadiOptiInitialize(obj, opti);
        
        % Casadi setters
        [obj, opti] = casadiInstantiateOptiParameters(obj, numericLiftingEnvironment, opti);
        
    end
end

