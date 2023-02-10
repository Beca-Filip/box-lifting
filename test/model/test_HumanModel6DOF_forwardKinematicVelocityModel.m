%TEST_HUMANMODEL6DOF_FORWARDKINEMATICVELOCITYMODEL makes a default 
%HumanModel6DOF with default KinematicPointsOfInterest6DOF (using 
%setDefaultKinematicPointsOfInterest) and calls the 
%forwardKinematicVelocityModel.
%
%   Expected output:
%
% ans =
% 
%          0    0.0000    0.0000    0.0000    0.0000    0.7071    1.7071    0.0000
%          0    1.0000    2.0000    3.0000    4.0000    3.2929    3.2929    4.0000
%          0         0         0         0         0         0         0         0
% 
% 
% ans =
% 
%          0   -1.0000   -2.0000   -3.0000   -4.0000   -3.2929   -3.2929   -4.0000
%          0    0.0000    0.0000    0.0000    0.0000    0.7071    1.7071    0.0000
% 
% 
% ans =
% 
%          0         0   -1.0000   -2.0000   -3.0000   -2.2929   -2.2929   -3.0000
%          0         0    0.0000    0.0000    0.0000    0.7071    1.7071    0.0000
% 
% 
% ans =
% 
%          0         0         0   -1.0000   -2.0000   -1.2929   -1.2929   -2.0000
%          0         0         0    0.0000    0.0000    0.7071    1.7071    0.0000
% 
% 
% ans =
% 
%          0         0         0         0   -1.0000   -0.2929   -0.2929   -1.0000
%          0         0         0         0    0.0000    0.7071    1.7071    0.0000
% 
% 
% ans =
% 
%          0         0         0         0         0    0.7071    0.7071         0
%          0         0         0         0         0    0.7071    1.7071         0
% 
% 
% ans =
% 
%      0     0     0     0     0     0     0     0
%      0     0     0     0     0     0     1     0

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

% Joint 1
dq = [1; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]

% Joint 2
dq = [0; % Ankle
     1;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]

% Joint 3
dq = [0; % Ankle
     0;    % Knee
     1;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]

% Joint 4
dq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     1;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]

% Joint 5
dq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     1;   % Shoulder
     0;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]

% Joint 6
dq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     1;      % Elbow
     ];
 
V = H.forwardKinematicVelocityModel(q,dq);
[V{:}]
