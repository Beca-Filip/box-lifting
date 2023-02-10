classdef KinematicPointOfInterest6DOF
    %KINEMATICPOINTOFINTEREST6DOF is a point which is rigidly attached to a link of
    %a HumanModel6DOF.
    
    properties (SetAccess = private)
        % All properties
        
        % The name of the point of interest (e.g. "Link 2 Frame Origin")
        Name(1, 1) string
        
        % The ordering number of the link the point is rigidly attached to
        % (e.g. 0 is the base frame, 6 is the wrist frame)
        RigidlyLinkedTo(1, 1) {mustBeInteger} = 0
        
        % The ordering number of the link frame in which the point's
        % position is constant (e.g. in box-lifting, the box is rigidly
        % attached to the wrist, but the position of the box changes less
        % in the global frame than in the wrist frame, so it makes sense to
        % express it as a constant in global frame)
        PositionExpressedInFrame(1, 1) {mustBeInteger} = 0
        
        % The position vector of the of the point of interest
        p(3, 1)
    end
    
    methods
        function obj = KinematicPointOfInterest6DOF(name,rigidlink,frame,pos)
            % KINEMATICPOINTOFINTEREST6DOF Construct an instance of this class
            %   Must be given a name, a link to which it is attached to, a
            %   frame in which the position vector is expressed, and the
            %   position vector.
            if (nargin == 0)
                name = "";
                rigidlink = 0;
                frame = 0;
                pos = zeros(3, 1);
            end
            obj.Name = name;
            obj.RigidlyLinkedTo = rigidlink;
            obj.PositionExpressedInFrame = frame;
            obj.p = pos;
        end
    end
end

