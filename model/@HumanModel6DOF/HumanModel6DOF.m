classdef HumanModel6DOF
    %HUMANMODEL6DOF creates a symmetric 6DOF planar model of the human. 
    %This class encapsulates the properties of the said model as well as the
    %functionalities of the Forward Kinematic Model and the Inverse Dynamic
    %Model.
    %
    %Base frame is assumed at the ankle, with X axis along the
    %posterior-anterior direction, and the Y axis along the
    %superior-inferior direction.
    %
    %Assumes a Modified-Denavit-Hartenberg base and link frame distribution.
    %The X axis of the base frame is along the posterior-anterior direction of
    %the body, and the Y axis is along the inferior-superior direction.
    %Consequently the Z axis is in the medial-lateral plane, along the
    %left-right direction.
    
    properties (Constant = true)
        % Tree Graph Properties
        
        nTreeLinks = 9; % Number of tree links
        TreeLinkNames = [ "Feet (2)", ...
                          "Shanks (2)", ...
                          "Thighs (2)", ...
                          "Abdomen (1)", ...
                          "Torso (1)", ...
                          "Upper Arms (2)", ...
                          "Forearms (2)", ...
                          "Hands (2)", ...
                          "Head (1)" ...
                        ]; % Names of tree links
                    
        % Ignoring feet, hands, and head
        % The Serial Representation

        nLinks = 6; % Number of serial links
        LinkNames = [ "Shanks (2)", ...
                      "Thighs (2)", ...
                      "Abdomen (1)", ...
                      "Torso (1)", ...
                      "Upper Arms (2)", ...
                      "Forearms (2)" ...
                    ]; % Names of tree links
        
        % Gravity        
        Gravity = -9.81;
    end
    
    properties (GetAccess = public, SetAccess = public)
        % Whole body properties
        
        WEIGHT(1, 1) % Full body weight [kg]
        HEIGHT(1, 1) % Full body height [m]
        
        % (Robot) base properties
        
        R(3,3) % Rotation matrix of the base frame w.r.t. global frame, units [m]
        p(3,1) % Position vector of the base frame w.r.t. global frame, units [m]
        
        HeelPosition(2, 1)  % 2D position of the heel in the base frame, units [m]
        ToePosition(2, 1)   % 2D position of the heel in the base frame, units [m]
                
        % Geometric and dynamic properties
        
        L(1,6)      % Link lengths, units [m]
        CoM(2,6)    % Link center of mass positions in local link frames, units [m]
        M(1,6)      % Mass scalars of the individual links, units [kg]
        Izz(1,6)    % Moment of inertia scalars about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        
        LFOOT(1, 1)  % FEET length, units [m]
        LHAND(1, 1)  % HAND length, units [m]
        LHEAD(1, 1)  % HEAD length, units [m]
        CoMFOOT(2, 1)  % FEET CoM position in local link frame, units [m]
        CoMHAND(2, 1)  % HEAD CoM position in local link frame, units [m]
        CoMHEAD(2, 1)  % HAND CoM position in local link frame, units [m]
        MFOOT(1, 1) % FEET mass [kg]
        MHAND(1, 1) % HAND mass [kg]
        MHEAD(1, 1) % HEAD mass [kg]
        IzzFOOT(1, 1) % FEET moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        IzzHAND(1, 1) % HAND moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        IzzHEAD(1, 1) % HEAD moment of inertia scalar about the Z axis of the local link frame centered at the CoM, units [kg.m^2]
        
%         MXYZ(2,6)   % First moment of link in local link frames, units [m.kg]
%         MXYZFOOT(2, 1) % FEET first moment in local link frame, units [m.kg]
%         MXYZHAND(2, 1) % HAND first moment in local link frame, units [m.kg]
%         MXYZHEAD(2, 1) % HEAD first moment in local link frame, units [m.kg]

        % Joint properties
        LowerJointLimits(6, 1)  % Lower limits of joint angles, [rad]
        UpperJointLimits(6, 1)  % Upper limits of joint angles, [rad]
        LowerTorqueLimits(6, 1) % Lower limits of joint torques, [N.m]
        UpperTorqueLimits(6, 1) % Upper limits of joint torques, [N.m]
        
        % Kinematic points of interest: Points w-e want to track
        LengthToRadiiFactor(1, 1)          % Scalar that determines the size of the collision spheres
        CS(1, :) CollisionSphere6DOF % Collision spheres, which will be tracked during FKM
        
        KPOI(1, :) KinematicPointOfInterest6DOF % Kinematic point of interest, which will be tracked during FKM
    end
    
    methods (Access = public)
        
        % Constructor
        function obj = HumanModel6DOF(R,p,WEIGHT,HEIGHT)
            %HUMANMODEL6DOF creates an object of this class. 
            %
            %If no arguments are passed, the constructor initializes the
            %properties to placeholder values: A 6DoF planar linkage in the 
            %floor plane R=I, p=0, which has homogeneous bars as links. 
            %Lenghts are 1m, masses are 1kg, CoMs are halfway along the link, 
            %inertias are computed by mL^2/12.
            
            if nargin == 0
                R = eye(3, 3);
                p = zeros(3, 1);
                L = ones(1, 6);
                M = ones(1, 6);
                CoM = 0.5 * [ones(1, 6); zeros(1, 6)];
%                 CoM = zeros(2, 6);
                Izz = M .* (L.^2) ./ 12;
                
                WEIGHT = sum(M);
                HEIGHT = sum(L(1:4));
                
                LFOOT = zeros(1, 1);
                LHAND = zeros(1, 1);
                LHEAD = zeros(1, 1);
                CoMFOOT = zeros(2, 1);
                CoMHEAD = zeros(2, 1);
                CoMHAND = zeros(2, 1);
                MFOOT = zeros(1, 1);
                MHAND = zeros(1, 1);
                MHEAD = zeros(1, 1);
                IzzFOOT = zeros(1, 1);
                IzzHAND = zeros(1, 1);
                IzzHEAD = zeros(1, 1);
                
%                 MXYZFOOT = zeros(2, 1);
%                 MXYZHAND = zeros(2, 1);
%                 MXYZHEAD = zeros(2, 1);
            
                obj.L = L;
                obj.CoM = CoM;
                obj.Izz = Izz;
                obj.M = M;
                obj.LFOOT = LFOOT;
                obj.LHAND = LHAND;
                obj.LHEAD = LHEAD;
                obj.CoMFOOT = CoMFOOT;
                obj.CoMHEAD = CoMHEAD;
                obj.CoMHAND = CoMHAND;
                obj.MFOOT = MFOOT;
                obj.MHAND = MHAND;
                obj.MHEAD = MHEAD;
                obj.IzzFOOT = IzzFOOT;
                obj.IzzHAND = IzzHAND;
                obj.IzzHEAD = IzzHEAD;
            end
            
            obj.R = R;
            obj.p = p;
            obj.WEIGHT = WEIGHT;
            obj.HEIGHT = HEIGHT;
            obj.LengthToRadiiFactor = 0.5;
            
            if nargin > 0
                obj = obj.setInertialParametersFromAnthropometricTables();
            end
            
            [LowerJointLimits, UpperJointLimits] = obj.defaultJointLimits();
            obj.LowerJointLimits = LowerJointLimits;
            obj.UpperJointLimits = UpperJointLimits;
            
            [LowerTorqueLimits, UpperTorqueLimits] = obj.defaultTorqueLimits();
            obj.LowerTorqueLimits = LowerTorqueLimits;
            obj.UpperTorqueLimits = UpperTorqueLimits;
            
            [HeelPosition, ToePosition] = obj.defaultHeelAndToePositions();
            obj.HeelPosition = HeelPosition;
            obj.ToePosition = ToePosition;
        end
        
        % Initializers
        [obj, opti] = defaultCasadiOptiInitialize(obj, opti);
        
        % Casadi setters
        [obj, opti] = casadiInstantiateOptiParameters(obj, numericLiftingEnvironment, opti);
        
        % Property modifiers
        obj = addKinematicPointOfIntrerest(obj, poi);
        obj = addCollisionSphere(obj, cs);
        
        % Multiple property setters
        obj = setDefaultKinematicPointsOfInterest(obj);
        obj = setDefaultCollisionSpheres(obj);
        obj = setInertialParametersFromAnthropometricTables(obj, weight, height);
        
        % Information getters
        [LowerJointLimits, UpperJointLimits] = defaultJointLimits(obj);
        [LowerTorqueLimits, UpperTorqueLimits] = defaultTorqueLimits(obj);
        [HeelPosition, ToePosition] = defaultHeelAndToePositions(obj);
        
        % Model computation methods
        PTS =  forwardKinematicModel(obj, q);
        [V, J] = forwardKinematicVelocityModel(obj,q,dq);
        [A, J, dJ] = forwardKinematicAccelerationModel(obj,q,dq, ddq);
        
        CTR =  collisionForwardKinematicModel(obj, q);
        D   =  collisionSpheresDistances(obj, q);
        [tau, f_grf] =  inverseDynamicModel(obj, q, dq, ddq, fFOOT, fHAND);
    end
end