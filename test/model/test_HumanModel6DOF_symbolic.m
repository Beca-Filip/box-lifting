%TEST_HUMANMODEL6DOF_SYMBOLIC makes a HumanModel6DOF with symbolic property
%values.
%
%   Expected output:
%
%     HumanModel6DOF with properties:
% 
%       R: [3×3 casadi.SX]
%       p: [3×1 casadi.SX]
%       L: [1×6 casadi.SX]
%       M: [1×6 casadi.SX]
%     CoM: [2×6 casadi.SX]
%     Izz: [1×6 casadi.SX]

H = HumanModel6DOF();

H.R = casadi.SX.sym('R', 3, 3);
H.p = casadi.SX.sym('p', 3, 1);
H.L = casadi.SX.sym('L', 1, 6);

H.CoM = casadi.SX.sym('CoM', 2, 6);
H.Izz = casadi.SX.sym('Izz', 1, 6);
H.M = casadi.SX.sym('M', 1, 6);

H = H.setDefaultKinematicPointsOfInterest();

disp(H)

q = [pi/2; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     -3*pi/4;   % Shoulder
     pi/4;      % Elbow
     ];
 
PTS = H.forwardKinematicModel(q);

[PTS{1}]
[PTS{2}]
[PTS{3}]
[PTS{4}]
[PTS{5}]
[PTS{6}]
[PTS{7}]
[PTS{8}]