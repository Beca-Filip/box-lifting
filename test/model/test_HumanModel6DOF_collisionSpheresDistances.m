%TEST_HUMANMODEL6DOF_COLLISIONSPHERESDISTANCES makes a default HumanModel6DOF
%with default CollisionSpheres (using setDefaultCollisionSpheres) and calls
%the collisionSpheresDistances.
%
%   Expected output:
% 
% ans =
% D =
% 
%   9×9 cell array
% 
%     {0×0 double}    {[       0]}    {[       1]}    {[       2]}    {[  2.1662]}    {[  2.0426]}    {[  2.7733]}    {[         3]}    {[  1.5414]}
%     {0×0 double}    {0×0 double}    {[       0]}    {[       1]}    {[  1.1754]}    {[  1.1614]}    {[  1.9756]}    {[         2]}    {[  1.8541]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {[       0]}    {[  0.1997]}    {[  0.4442]}    {[  1.3823]}    {[         1]}    {[  2.4051]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {[ -0.6173]}    {[  0.2247]}    {[  1.2196]}    {[         0]}    {[  3.1098]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {[ -0.0761]}    {[  0.8990]}    {[1.1102e-16]}    {[  3.0056]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {[       0]}    {[    0.8990]}    {[  2.2493]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {[    1.8478]}    {[  2.5376]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double  }    {[       4]}
%     {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double}    {0×0 double  }    {0×0 double}

H = HumanModel6DOF();

H = H.setDefaultCollisionSpheres();


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

[H.CS.radius]

D = H.collisionSpheresDistances(q);
D