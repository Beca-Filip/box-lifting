function sphere = createBoxCollisionSphere(obj)

name = "Box Sphere";
rigidlink = 7;
frame = 0;
pos = [obj.WristToBoxGripPointVector; 0];
radius = max(obj.BoxWidth/2,obj.BoxHeight/2);


sphere = CollisionSphere6DOF(name,rigidlink,frame,pos,radius);
end