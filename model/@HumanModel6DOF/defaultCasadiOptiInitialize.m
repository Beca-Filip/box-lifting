function [obj, opti] = defaultCasadiOptiInitialize(obj, opti)
%DEFAULTCASADIOPTIINITIALIZE returns a HumanModel6DOF object with default
%casadi opti initializers.
%
%   [obj, opti] = DEFAULTCASADIOPTIINITIALIZE(opti)
%   

% Get all default values
WEIGHT = opti.parameter(1, 1);
HEIGHT = opti.parameter(1, 1);

R = opti.parameter(3, 3);
p = opti.parameter(3, 1);

HeelPosition = opti.parameter(2, 1);
ToePosition = opti.parameter(2, 1);

L = opti.parameter(1, 6);
CoM = opti.parameter(2,6);
M = opti.parameter(1,6);
Izz = opti.parameter(1,6);

LFOOT = opti.parameter(1, 1);
LHAND = opti.parameter(1, 1);
LHEAD = opti.parameter(1, 1);
CoMFOOT = opti.parameter(2, 1);
CoMHAND = opti.parameter(2, 1);
CoMHEAD = opti.parameter(2, 1);
MFOOT = opti.parameter(1, 1);
MHAND = opti.parameter(1, 1);
MHEAD = opti.parameter(1, 1);
IzzFOOT = opti.parameter(1, 1);
IzzHAND = opti.parameter(1, 1);
IzzHEAD = opti.parameter(1, 1);

LowerJointLimits = opti.parameter(6, 1);
UpperJointLimits = opti.parameter(6, 1);
LowerTorqueLimits = opti.parameter(6, 1);
UpperTorqueLimits = opti.parameter(6, 1);

LengthToRadiiFactor = opti.parameter(1, 1);

% Just store them in the object
obj.WEIGHT = WEIGHT;
obj.HEIGHT = HEIGHT;

obj.R = R;
obj.p = p;

obj.HeelPosition = HeelPosition;
obj.ToePosition = ToePosition;

obj.L = L;
obj.CoM = CoM;
obj.M = M;
obj.Izz = Izz;
obj.LFOOT = LFOOT;
obj.LHAND = LHAND;
obj.LHEAD = LHEAD;
obj.CoMFOOT = CoMFOOT;
obj.CoMHAND = CoMHAND;
obj.CoMHEAD = CoMHEAD;
obj.MFOOT = MFOOT;
obj.MHAND = MHAND;
obj.MHEAD = MHEAD;
obj.IzzFOOT = IzzFOOT;
obj.IzzHAND = IzzHAND;
obj.IzzHEAD = IzzHEAD;

obj.LowerJointLimits = LowerJointLimits;
obj.UpperJointLimits = UpperJointLimits;
obj.LowerTorqueLimits = LowerTorqueLimits;
obj.UpperTorqueLimits = UpperTorqueLimits;

obj.LengthToRadiiFactor = LengthToRadiiFactor;

% Get the kinematic and collision points
obj = setDefaultKinematicPointsOfInterest(obj);
obj = setDefaultCollisionSpheres(obj);
end

