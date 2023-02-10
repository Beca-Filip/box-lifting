%TEST_HUMANMODEL6DOF_FORWARDKINEMATICACCELERATIONMODEL makes a default 
%HumanModel6DOF with default KinematicPointsOfInterest6DOF (using 
%setDefaultKinematicPointsOfInterest) and calls the 
%forwardKinematicAccelerationModel.
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

% Position
q = [pi/2; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     -3*pi/4;   % Shoulder
     pi/4;      % Elbow
     ];
% FKM
% PTS = H.forwardKinematicModel(q);
% [PTS{:}]

% Velocity
dq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
% FKMV
% [V, Jv] = H.forwardKinematicVelocityModel(q, dq);
% [V{:}]

% Acceleration
% Joint 1
ddq = [1; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

% Joint 2
ddq = [0; % Ankle
     1;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

% Joint 3
ddq = [0; % Ankle
     0;    % Knee
     1;    % Hip
     0;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

% Joint 4
ddq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     1;    % Back
     0;   % Shoulder
     0;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

% Joint 5
ddq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     1;   % Shoulder
     0;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

% Joint 6
ddq = [0; % Ankle
     0;    % Knee
     0;    % Hip
     0;    % Back
     0;   % Shoulder
     1;      % Elbow
     ];
 
A = H.forwardKinematicAccelerationModel(q,dq,ddq);
[A{:}]

 