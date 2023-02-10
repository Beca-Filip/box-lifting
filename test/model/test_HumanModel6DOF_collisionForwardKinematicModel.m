%TEST_HUMANMODEL6DOF_COLLISIONFORWARDKINEMATICMODEL makes a default HumanModel6DOF
%with default CollisionSpheres (using setDefaultCollisionSpheres) and calls
%the collisionForwardKinematicModel.
%
%   Expected output:
% 
% ans =
% 
%     0.0000    0.0000    0.0000    0.0000    0.3536    1.2071    1.7071    0.0000    3.0000
%     0.5000    1.5000    2.5000    3.5000    3.6464    3.2929    3.2929    4.0000         0
%          0         0         0         0         0         0         0         0         0

H = HumanModel6DOF();

lenghtToRadiiFactor = 0.25;
H = H.setDefaultCollisionSpheres(lenghtToRadiiFactor);


% Create one more collision sphere
name = "Outer";
rigidlink = 0;
frame = 0;
pos = [3;0;0];
radius = 1;
S = CollisionSphere6DOF(name,rigidlink,frame,pos,radius);

% Add it
H = H.addCollisionSphere(S);

q = [pi/2; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     -3*pi/4;   % Shoulder
     pi/4;      % Elbow
     ];
 
PTS = H.collisionForwardKinematicModel(q);

[PTS{:}]