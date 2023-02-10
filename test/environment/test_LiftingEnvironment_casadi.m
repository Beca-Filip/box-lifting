%TEST_LIFTINGENVIRONMENT_CASADI is a script that test the initialization
%of the environment with CASADI opti object.


% Create object
opti = casadi.Opti();


% Create casadi environment
casadiLiftingEnvironment = LiftingEnvironment();
[casadiLiftingEnvironment, opti] = casadiLiftingEnvironment.defaultCasadiOptiInitialize(opti);

% Create numeric environment
numericLiftingEnvironment2 = LiftingEnvironment();

% Display opti object
disp(opti)

% Initialize casadi from numeric
[casadiLiftingEnvironment, opti] = casadiLiftingEnvironment.casadiInstantiateOptiParameters(numericLiftingEnvironment2, opti);

% Print
disp(opti)