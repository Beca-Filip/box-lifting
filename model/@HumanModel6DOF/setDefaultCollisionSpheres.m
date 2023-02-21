function obj = setDefaultCollisionSpheres(obj)
    %SETDEFAULTCOLLISIONSPHERES creates the default collision spheres as 
    %centered at the link CoMs with radii equal to a scalar times the link
    %length.
    %
    %     obj = SETDEFAULTCOLLISIONSPHERES(obj)
    % 
    
    % From Shank to Forearm link origins
    for link = 1 : 6                
        name = sprintf("%s (Link %d) CoM", obj.LinkNames(link), link);
        rigidlink = link;
        frame = link;
        pos = [obj.CoM(:, link); 0];
        radius = obj.L(link) / 2 * obj.LengthToRadiiFactor;
        
        S = CollisionSphere6DOF(name, rigidlink, frame, pos, radius);
        obj.CS = [obj.CS, S];
    end
    
%     % Hand link origin
%     name = sprintf("%s (Link 8) CoM", obj.TreeLinkNames(8));
%     rigidlink = 6;
%     frame = 6;
%     pos = [obj.L(6); zeros(2, 1)] + [obj.CoMHAND; 0];
%     radius = obj.LHAND * obj.LengthToRadiiFactor;
%     
%     S = CollisionSphere6DOF(name, rigidlink, frame, pos, radius);
%     obj.CS = [obj.CS, S];
%     
%     % Head link origin
%     name = sprintf("%s (Link 9) CoM", obj.TreeLinkNames(9));
%     rigidlink = 4;
%     frame = 4;
%     pos = [obj.L(4); zeros(2, 1)] + [obj.CoMHEAD; 0];
%     radius = obj.LHEAD * obj.LengthToRadiiFactor;
%     
%     S = CollisionSphere6DOF(name, rigidlink, frame, pos, radius);
%     obj.CS = [obj.CS, S];
end