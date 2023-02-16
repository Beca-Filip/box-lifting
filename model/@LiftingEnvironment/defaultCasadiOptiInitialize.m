function [obj, opti] = defaultCasadiOptiInitialize(obj, opti)
%DEFAULTCASADIOPTIINITIALIZE returns a LiftingEnvironment object with default
%casadi opti initializers.
%
%   [obj, opti] = DEFAULTCASADIOPTIINITIALIZE(obj, opti)
%   

% Get all default values
BoxWidth = opti.parameter(1, 1);
BoxHeight = opti.parameter(1, 1);
BoxMass = opti.parameter(1, 1);

TableWidth = opti.parameter(1, 1);
TableHeight = opti.parameter(1, 1);
TableCenterCoordinates = opti.parameter(2, 1);

WristInitialPosition = opti.parameter(2, 1);
WristFinalPosition = opti.parameter(2, 1);

WristToBoxGripPointVector = opti.parameter(2, 1);

JointAnglesInitial = opti.parameter(6, 1);
JointAnglesFinal = opti.parameter(6, 1);

% Just store them in the object
obj.BoxWidth = BoxWidth;
obj.BoxHeight = BoxHeight;
obj.BoxMass = BoxMass;
obj.TableWidth = TableWidth;
obj.TableHeight = TableHeight;
obj.TableCenterCoordinates = TableCenterCoordinates;
obj.WristInitialPosition = WristInitialPosition;
obj.WristFinalPosition = WristFinalPosition;
obj.WristToBoxGripPointVector = WristToBoxGripPointVector;
obj.JointAnglesInitial = JointAnglesInitial;
obj.JointAnglesFinal = JointAnglesFinal;

end

