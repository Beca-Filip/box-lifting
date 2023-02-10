%TEST_HUMANMODEL6DOF_SETDEFAULTCOLLISIONSPHERES makes a default 
%HumanModel6DOF and calls the setDefaultCollisionSpheres method.
%
%   Expected output:
%   Columns 1 through 6
% 
%     "Shanks (2) (Link …"    "Thighs (2) (Link …"    "Abdomen (1) (Link…"    "Torso (1) (Link 4…"    "Upper Arms (2) (L…"    "Forearms (2) (Lin…"
% 
%   Columns 7 through 8
% 
%     "Hands (2) (Link 8…"    "Head (1) (Link 9)…"
% 
%      1     2     3     4     5     6     6     4
% 
%      1     2     3     4     5     6     6     4
% 
%     0.5000    0.5000    0.5000    0.5000    0.5000    0.5000    1.0000    1.0000
%          0         0         0         0         0         0         0         0
%          0         0         0         0         0         0         0         0
% 
%     0.2500    0.2500    0.2500    0.2500    0.2500    0.2500         0         0


H = HumanModel6DOF();

lengthToRadiiFactor = 0.25;
H = H.setDefaultCollisionSpheres(lengthToRadiiFactor);

disp([H.CS.Name])
disp([H.CS.RigidlyLinkedTo])
disp([H.CS.PositionExpressedInFrame])
disp([H.CS.p])
disp([H.CS.radius])