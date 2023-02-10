function obj = setDefaultKinematicPointsOfInterest(obj)
    %SETDEFAULTKINEMATICPOINTSOFINTEREST creates the default points of 
    %interest as the link frame origins, and the end effector origin.
    
    % From Shank to Forearm link origins
    for link = 1 : 6                
        name = sprintf("%s (Link %d) frame origin", obj.LinkNames(link), link);
        rigidlink = link;
        frame = link;
        pos = zeros(3, 1);

        K = KinematicPointOfInterest6DOF(name, rigidlink, frame, pos);
        obj.KPOI = [obj.KPOI, K];
    end
    
    % Hand link origin
    name = sprintf("%s (Link 8) frame origin", obj.TreeLinkNames(8));
    rigidlink = 6;
    frame = 6;
    pos = [obj.L(6); zeros(2, 1)];
    
    K = KinematicPointOfInterest6DOF(name, rigidlink, frame, pos);
    obj.KPOI = [obj.KPOI, K];
    
    % Head link origin
    name = sprintf("%s (Link 9) frame origin", obj.TreeLinkNames(9));
    rigidlink = 4;
    frame = 4;
    pos = [obj.L(4); zeros(2, 1)];
    
    K = KinematicPointOfInterest6DOF(name, rigidlink, frame, pos);
    obj.KPOI = [obj.KPOI, K];
end