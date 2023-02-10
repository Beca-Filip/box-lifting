function [LowerJointLimits,UpperJointLimits] = defaultJointLimits(obj)
%DEFAULTJOINTLIMITS returns the default lower and upper joint limits for
%the 6DOF homan model.
%The numerical limits were taken from Robert et al. (2013).
%
%   [LowerJointLimits,UpperJointLimits] = DEFAULTJOINTLIMITS(obj)
%   

% Torque Limits: hard coded
LowerJointLimits = [...
    0;      % Ankle plantarflexion angle [rad]
    0;          % Knee flexion angle [rad]
    -pi;        % Hip extension angle [rad]
    -pi/3;      % Lumbar flexion angle [rad]
    -5*pi/6;    % Shoulder extension angle [rad]
    0;          % Elbow extension angle [rad]    
];

UpperJointLimits = [...
    pi/2;       % Ankle dorsiflexion angle [rad]
    pi;         % Knee extension angle [rad]
    pi/6;       % Hip flexion angle [rad]
    pi/3;       % Lumbar extension angle [rad]
    pi/6;       % Shoulder flexion angle [rad]
    pi;         % Elbow flexion angle [rad]
];
end

