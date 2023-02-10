%TEST_HUMANMODEL6DOF_INVERSEDYNAMICMODEL makes a default HumanModel6DOF
%with default KinematicPointsOfInterest6DOF (using 
%setDefaultKinematicPointsOfInterest) and calls the inverseDynamicModel.
%
%   Expected output:
%     ans =
% 
%              0    0.0000    0.0000    0.0000    0.0000    0.7071    1.7071    0.0000
%              0    1.0000    2.0000    3.0000    4.0000    3.2929    3.2929    4.0000
%              0         0         0         0         0         0         0         0
% 
% 
%     tau =
% 
%         6.9367
%         6.9367
%         6.9367
%         6.9367
%         6.9367
%              0
% 
% 
%     f_grf =
% 
%        -0.0000
%       -58.8600
%              0
%              0
%              0
%        -6.9367


H = HumanModel6DOF();

H = H.setDefaultKinematicPointsOfInterest();

% Position
q = [pi/2; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     -3*pi/4;   % Shoulder
     pi/4;      % Elbow
     ];

% Forward Kinematic Model
PTS = H.forwardKinematicModel(q);
[PTS{:}]


% Velocity
dq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];

% Acceleration
ddq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
% No external forces
fFOOT = zeros(6, 1);
fHAND = zeros(6, 1);

% Calculate dynamics
[tau, f_grf] = H.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND);

tau
f_grf

% External forces of 1N along Y axis at the hand
fFOOT = zeros(6, 1);
fHAND = [0; 1; zeros(4, 1)];

% Calculate dynamics
[tau, f_grf] = H.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND);

tau
f_grf