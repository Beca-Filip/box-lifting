%TEST_HUMANMODEL6DOF_CASADI tests the casadi opti functions of the human
%model.

opti = casadi.Opti();

% Create symbolic model
casadiHumanModel = HumanModel6DOF();
[casadiHumanModel, opti] = casadiHumanModel.defaultCasadiOptiInitialize(opti);

% Create numeric model
numericHumanModel = HumanModel6DOF(eye(3), zeros(3, 1), 80, 1.85);

% Print opti
disp(opti)

% Instantiate mode
[instantiatedHumanModel, opti] = casadiHumanModel.casadiInstantiateOptiParameters(numericHumanModel, opti);

disp(opti)