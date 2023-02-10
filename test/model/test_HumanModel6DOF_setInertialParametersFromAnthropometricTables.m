%TEST_HUMANMODEL6DOF_SETDEFAULTKINEMATICPOINTSOFINTEREST makes a default 
%HumanModel6DOF and calls the setDefaultKinematicPointsOfInterest method.
clc
H = HumanModel6DOF();

R = eye(3);
p = zeros(3, 1);
WEIGHT = 88;
HEIGHT = 1.88;

H = H.setInertialParametersFromAnthropometricTables(WEIGHT, HEIGHT);