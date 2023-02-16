function [LowerJointLimits,UpperJointLimits] = defaultJointLimits(obj)
%DEFAULTJOINTLIMITS returns the default lower and upper joint limits for
%the 6DOF homan model.
%The numerical limits were taken from Robert et al. (2013).
%
%   [LowerJointLimits,UpperJointLimits] = DEFAULTJOINTLIMITS(obj)
%   

% Torque Limits: hard coded
LowerJointLimits = [...
    0;          % Ankle plantarflexion angle [rad]
    -pi/18;     % Knee flexion angle [rad]
    -pi;        % Hip extension angle [rad]
    -2*pi/5;      % Lumbar flexion angle [rad]
    -3*pi/2;    % Shoulder extension angle [rad]
    -pi/18;     % Elbow extension angle [rad]    
];

UpperJointLimits = [...
    3*pi/4;     % Ankle dorsiflexion angle [rad]
    pi;         % Knee extension angle [rad]
    pi/10;      % Hip flexion angle [rad]
    pi/4;       % Lumbar extension angle [rad]
    pi/12;      % Shoulder flexion angle [rad]
    pi;         % Elbow flexion angle [rad]
];
end

