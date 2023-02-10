classdef CollisionSphere6DOF 
    %COLLISIONSPHERE6DOF is a sphere which is rigidly attached to a link of
    %a HumanModel6DOF.
    
    properties (SetAccess = private)
        % All properties
        
        % The name of the collision sphere (e.g. "Link 2 Frame Origin")
        Name(1, 1) string
        
        % The ordering number of the link the sphere is rigidly attached to
        % (e.g. 0 is the base frame, 6 is the wrist frame)
        RigidlyLinkedTo(1, 1) {mustBeInteger} = 0
        
        % The ordering number of the link frame in which the sphere's
        % position is constant (e.g. in box-lifting, the box is rigidly
        % attached to the wrist, but the position of the box changes less
        % in the global frame than in the wrist frame, so it makes sense to
        % express it as a constant in global frame)
        PositionExpressedInFrame(1, 1) {mustBeInteger} = 0
        
        % The position vector of the of the center of the collision sphere
        p(3, 1)
        
        % The radius of the collision sphere
        radius(1, 1) = 1
    end
    
    methods
        function obj = CollisionSphere6DOF(name,rigidlink,frame,pos,radius)
            % COLLISIONSPHERE6DOF Construct an instance of this class.
            %   Must be given a name, a link to which it is attached to, a
            %   frame in which the position vector is expressed, the
            %   position vector of the center in this frame, and the radius
            %   of the sphere.
            if (nargin == 0)
                name = "";
                rigidlink = 0;
                frame = 0;
                pos = zeros(3, 1);
                radius = 1;
            end
            obj.Name = name;
            obj.RigidlyLinkedTo = rigidlink;
            obj.PositionExpressedInFrame = frame;
            obj.p = pos;
            obj.radius = radius;
        end
    end
end

