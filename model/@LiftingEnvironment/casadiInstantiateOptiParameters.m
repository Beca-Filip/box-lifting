function [obj, opti] = casadiInstantiateOptiParameters(obj, numericLiftingEnvironment, opti)
%CASADIINSTANTIATEOPTIPARAMETERS returns an casadi.Opti object where the
%LiftingEnvironment parameters have been instantiated with numerical
%values copied from a numeric lifting environment object.
%
%   opti = CASADIINSTANTIATEOPTIPARAMETERS(obj, numericLiftingEnvironment, opti)
%   

% Set values for the opti parameters
opti.set_value(obj.BoxWidth, numericLiftingEnvironment.BoxWidth);
opti.set_value(obj.BoxHeight, numericLiftingEnvironment.BoxHeight);
opti.set_value(obj.BoxMass, numericLiftingEnvironment.BoxMass);
opti.set_value(obj.TableWidth, numericLiftingEnvironment.TableWidth);
opti.set_value(obj.TableHeight, numericLiftingEnvironment.TableHeight);
opti.set_value(obj.TableCenterCoordinates, numericLiftingEnvironment.TableCenterCoordinates);
opti.set_value(obj.WristInitialPosition, numericLiftingEnvironment.WristInitialPosition);
opti.set_value(obj.WristFinalPosition, numericLiftingEnvironment.WristFinalPosition);
opti.set_value(obj.WristToBoxGripPointVector, numericLiftingEnvironment.WristToBoxGripPointVector);
opti.set_value(obj.JointAnglesInitial, numericLiftingEnvironment.JointAnglesInitial);
opti.set_value(obj.JointAnglesFinal, numericLiftingEnvironment.JointAnglesFinal);
end

