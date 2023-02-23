function spheres = createTableCollisionSphere(obj)

% Same for all spheres
rigidlink = 0;
frame = 0;

% Spheres array
spheres = [];

% Radius of the spheres is fixed to width/10
radius = obj.TableWidth / 10;

% Table upper corner
TableUpperCornerCoordinates = obj.TableCenterCoordinates + [-obj.TableWidth/2; obj.TableHeight/2];

% Create five spheres along the upper edge of the table
for nSph = 1 : 5
    name = sprintf("Upper-edge table sphere %d", nSph);
    sphCenter = TableUpperCornerCoordinates + [(nSph-1)*2*radius + radius; -radius];
    sphCenter = [sphCenter; 0];
    spheres = [spheres, CollisionSphere6DOF(name,rigidlink,frame,sphCenter,radius)];
end

end