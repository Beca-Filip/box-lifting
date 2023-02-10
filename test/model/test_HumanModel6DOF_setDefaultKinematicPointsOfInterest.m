%TEST_HUMANMODEL6DOF_SETDEFAULTKINEMATICPOINTSOFINTEREST makes a default 
%HumanModel6DOF and calls the setDefaultKinematicPointsOfInterest method.
%
%   Expected output:
%   Columns 1 through 7
% 
%     "Shanks (2) (Link …"    "Thighs (2) (Link …"    "Abdomen (1) (Link…"    "Torso (1) (Link 4…"    "Upper Arms (2) (L…"    "Forearms (2) (Lin…"    "Hands (2) (Link 8…"
% 
%   Column 8
% 
%     "Head (1) (Link 9)…"
% 
%      1     2     3     4     5     6     6     4
% 
%      1     2     3     4     5     6     6     4
% 
%      0     0     0     0     0     0     1     1
%      0     0     0     0     0     0     0     0
%      0     0     0     0     0     0     0     0
H = HumanModel6DOF();

H = H.setDefaultKinematicPointsOfInterest();

disp([H.KPOI.Name])
disp([H.KPOI.RigidlyLinkedTo])
disp([H.KPOI.PositionExpressedInFrame])
disp([H.KPOI.p])