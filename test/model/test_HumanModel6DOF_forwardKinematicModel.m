%TEST_HUMANMODEL6DOF_FORWARDKINEMATICMODEL makes a default HumanModel6DOF
%with default KinematicPointsOfInterest6DOF (using 
%setDefaultKinematicPointsOfInterest) and calls the forwardKinematicModel.
%
%   Expected output:
%
% 
%     ans =
% 
%              0    0.0000    0.0000    0.0000    0.0000    0.7071    1.7071
%              0    1.0000    2.0000    3.0000    4.0000    3.2929    3.2929
%              0         0         0         0         0         0         0

H = HumanModel6DOF();

H = H.setDefaultKinematicPointsOfInterest();

q = [pi/2; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     -3*pi/4;   % Shoulder
     pi/4;      % Elbow
     ];
 
PTS = H.forwardKinematicModel(q);

[PTS{:}]