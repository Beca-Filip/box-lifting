%TEST_LIFTINGENVIRONMENT_CASADI is a script that test the initialization
%of the environment with CASADI opti object.


% Create object
opti = casadi.Opti();

%% Human model
% Create symbolic model
casadiHumanModel = HumanModel6DOF();
[casadiHumanModel, opti] = casadiHumanModel.defaultCasadiOptiInitialize(opti);

%% LiftingEnvironment
% Create casadi environment
casadiLiftingEnvironment = LiftingEnvironment();
[casadiLiftingEnvironment, opti] = casadiLiftingEnvironment.defaultCasadiOptiInitialize(opti);

%% Trajectory
% Spline trajectory parameters
dimension = 6;
degree = 5;
knotNumber = 10;
evaluationNumber = 100;
[casadiTrajectory, opti] = SplineTrajectory.defaultCasadiOptiInitialize(opti, dimension, degree, knotNumber, evaluationNumber);
