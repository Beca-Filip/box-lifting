clear all;
close all;
clc;

% Create a DOC instance
Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 100;

doc = BoxLiftingDOC(splineDegree, splineNumKnots, splineEvaluationNumber, "constant_final_velocities", "time_changing_parameters");
% Constraints
doc = doc.addInitialJointConstraints();
doc = doc.addFinalCartesianConstraints();
doc = doc.addJointLimitConstraints();
doc = doc.addTorqueLimitConstraints();
doc = doc.addCopLimitConstraints();
doc = doc.addCollisionConstraints();
% Costs
doc = doc.addSumSquaredJointVelocitiesCost();
doc = doc.addSumSquaredJointAccelerationsCost();
doc = doc.addSumSquaredJointJerksCost();
doc = doc.addSumSquaredWristVelocityCost();
doc = doc.addSumSquaredWristAccelerationCost();
doc = doc.addSumSquaredJointTorquesCost();
doc = doc.addSumSquaredJointPowersCost();
% Create parameterized cost function
doc = doc.setParametrizedCostFunction();

% Initialize
TrialIndex = 1;
Trials(TrialIndex).splineTrajectory = Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
doc = doc.instantiateParameters(Trials(TrialIndex).humanModel, Trials(TrialIndex).liftingEnvironment, Trials(TrialIndex).splineTrajectory);

% DOC options
sol_opt = struct;
sol_opt.ipopt.print_level = 0;
sol_opt.print_time =0;
sol_opt.verbose = 0;
sol_opt.ipopt.sb ='yes';
sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
sol_opt.regularity_check = true;

% Set DOC options
doc.opti.solver('ipopt', sol_opt);


% Which trials we look at
bllioc = BoxLiftingLocalIOC(Trials(TrialIndex), doc);
% Test initial weights IOC
w0 = bllioc.getInitialWeightsIOC(length(doc.costFunctionVector), 3);
% Test initial weights DOC
w1 = bllioc.expandWeightsForDOC(w0);

% Set cost function parameters
doc = doc.setCostFunctionParameters(w1);

%% Test run function
fprintf("Started IOC.\n")
nT = 3;
optimopts = optimoptions(...
    @fmincon, ...
    'Algorithm', 'interior-point', ...
    'Display', 'iter-detailed', ...
    'MaxIterations', 100, ...
    'PlotFcn', {'optimplotfval', 'optimplotstepsize'} ...
);
bllioc = bllioc.runIOC(nT, optimopts);
fprintf("Ended IOC.\n");