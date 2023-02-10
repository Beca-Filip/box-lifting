function [obj, opti] = casadiInstantiateOptiParameters(obj, numericHumanModel, opti)
%CASADIINSTANTIATEOPTIPARAMETERS returns an casadi.Opti object where the
%HumanModel6DOF parameters have been instantiated with numerical
%values copied from a numeric human model object.
%
%   opti = CASADIINSTANTIATEOPTIPARAMETERS(obj, numericHumanModel, opti)
%   

% Set values for the opti parameters
opti.set_value(obj.WEIGHT, numericHumanModel.WEIGHT);
opti.set_value(obj.HEIGHT, numericHumanModel.HEIGHT);

opti.set_value(obj.R, numericHumanModel.R);
opti.set_value(obj.p, numericHumanModel.p);

opti.set_value(obj.HeelPosition, numericHumanModel.HeelPosition);
opti.set_value(obj.ToePosition, numericHumanModel.ToePosition);

opti.set_value(obj.L, numericHumanModel.L);
opti.set_value(obj.CoM, numericHumanModel.CoM);
opti.set_value(obj.M, numericHumanModel.M);
opti.set_value(obj.Izz, numericHumanModel.Izz);
opti.set_value(obj.LFOOT, numericHumanModel.LFOOT);
opti.set_value(obj.LHAND, numericHumanModel.LHAND);
opti.set_value(obj.LHEAD, numericHumanModel.LHEAD);
opti.set_value(obj.CoMFOOT, numericHumanModel.CoMFOOT);
opti.set_value(obj.CoMHAND, numericHumanModel.CoMHAND);
opti.set_value(obj.CoMHEAD, numericHumanModel.CoMHEAD);
opti.set_value(obj.MFOOT, numericHumanModel.MFOOT);
opti.set_value(obj.MHAND, numericHumanModel.MHAND);
opti.set_value(obj.MHEAD, numericHumanModel.MHEAD);
opti.set_value(obj.IzzFOOT, numericHumanModel.IzzFOOT);
opti.set_value(obj.IzzHAND, numericHumanModel.IzzHAND);
opti.set_value(obj.IzzHEAD, numericHumanModel.IzzHEAD);

opti.set_value(obj.LowerJointLimits, numericHumanModel.LowerJointLimits);
opti.set_value(obj.UpperJointLimits, numericHumanModel.UpperJointLimits);
opti.set_value(obj.LowerTorqueLimits, numericHumanModel.LowerTorqueLimits);
opti.set_value(obj.UpperTorqueLimits, numericHumanModel.UpperTorqueLimits);

opti.set_value(obj.LengthToRadiiFactor, numericHumanModel.LengthToRadiiFactor);
end

