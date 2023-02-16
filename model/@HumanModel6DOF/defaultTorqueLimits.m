function [LowerTorqueLimits,UpperTorqueLimits] = defaultTorqueLimits(obj)
%DEFAULTTORQUELIMITS returns the default lower and upper torque limits for
%the 6DOF homan model.
%The numerical limits were taken from Robert et al. (2013).
%
%   [LowerTorqueLimits,ExtensionTorqueLimits] = DEFAULTTORQUELIMITS(obj)
%   

% Torque Limits: hard coded
LowerTorqueLimits = [...
    2 * 126;    % Ankle plantarflexion torque [N.m]
    2 * 168;    % Knee extension torque [N.m]
    2 * 190;    % Hip extension torque [N.m]
    143;        % Lumbar flexion torque [N.m]
    2 * 67;     % Shoulder extension torque [N.m]
    2 * 46;     % Elbow extension torque [N.m]    
];

LowerTorqueLimits = -LowerTorqueLimits;

UpperTorqueLimits = [...
    2 * 126;    % Ankle dorsiflexion torque [N.m]
    2 * 100;    % Knee flexion torque [N.m]
    2 * 185;    % Hip flexion torque [N.m]
    334;        % 234;        % Lumbar extension torque [N.m]
    2 * 92;     % Shoulder flexion torque [N.m]
    2 * 77;     % Elbow flexion torque
];
end

