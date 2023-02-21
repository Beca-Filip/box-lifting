function sphere = createTableCollisionSphere(obj)

name = "Table sphere";
rigidlink = 0;
frame = 0;
pos = obj.TableCenterCoordinates;
radius = max(obj.TableWidth/2, obj.TableHeight/2);


sphere = CollisionSphere6DOF(name,rigidlink,frame,pos,radius);
end