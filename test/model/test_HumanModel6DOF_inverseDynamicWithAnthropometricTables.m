%TEST_HUMANMODEL6DOF_INVERSEDYNAMICWITHANTHROPOMETRICTABLES makes a default 
%HumanModel6DOF and calls the setDefaultKinematicPointsOfInterest method.
clc
H = HumanModel6DOF();

R = eye(3);
p = zeros(3, 1);
WEIGHT = 88;
HEIGHT = 1.88;

H = H.setInertialParametersFromAnthropometricTables(WEIGHT, HEIGHT);

% Pose
q = [pi/2;  % Ankle angle [rad]
     0;     % Knee angle [rad]
     0;     % Hip angle [rad]
     0;     % Back angle [rad]
     -3*pi/4;    % Shoulder angle [rad]
     pi/4;     % Elbow angle [rad]
    ];
% Velocity
dq = [0;        % Ankle velocity [rad/s]
      0;        % Knee velocity [rad/s]
      0;        % Hip velocity [rad/s]
      0;        % Back velocity [rad/s]
      0;        % Shoulder velocity [rad/s]
      0;        % Elbow velocity [rad/s]
     ];
% Acceleration
ddq = [0;       % Ankle acceleration [rad.s^-2]
       0;       % Knee acceleration [rad.s^-2]
       0;       % Hip acceleration [rad.s^-2]
       0;       % Back acceleration [rad.s^-2]
       0;       % Shoulder acceleration [rad.s^-2]
       0;       % Elbow acceleration [rad.s^-2]
      ];
% External forces on the foot 
fFOOT = zeros(6, 1);
% External forces on the hand
fHAND = zeros(6, 1);

% Display joint moments and ground reaction forces
[tau, f_grf] = H.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND)

% Display the model
figure;
Animate_nDOF(q, H.L, 0.1);