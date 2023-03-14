clear all;
close all;
clc;

% Create a DOC instance
Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 100;

doc = BoxLiftingDOC(splineDegree, splineNumKnots, splineEvaluationNumber, "constant_final_velocities", "constant_parameters");
% Constraints
doc = doc.addInitialJointConstraints();
doc = doc.addFinalCartesianConstraints();
doc = doc.addJointLimitConstraints();
doc = doc.addTorqueLimitConstraints();
doc = doc.addCopLimitConstraints();
doc = doc.addCollisionConstraints();
% Costs
doc = doc.addSumSquaredJointVelocitiesCost(0.82);
doc = doc.addSumSquaredJointAccelerationsCost(10.61);
doc = doc.addSumSquaredJointJerksCost(377.52);
doc = doc.addSumSquaredWristVelocityCost(0.17);
doc = doc.addSumSquaredWristAccelerationCost(1.16);
doc = doc.addSumSquaredJointTorquesCost(90.39);
doc = doc.addSumSquaredJointPowersCost(41.44);
% Create parameterized cost function
doc = doc.setParametrizedCostFunction();

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

% Get Trial Indices
% TrialIndices = (0 : 20 : 100) + [1;];
% TrialIndices = (0 : 20 : 100) + [1;6;11;16];
TrialIndices = [1;6;11;16];
TrialIndices = TrialIndices(:);

% Resample spline trajectory for these trial indices
for ii = 1 : length(TrialIndices)
    TrialIndex = TrialIndices(ii);
    Trials(TrialIndex).splineTrajectory = ...
    Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration), splineDegree-1);
end

% Which trials we look at
bllioc = BoxLiftingLocalIOC(Trials(TrialIndices), doc);

%% Test run function
fprintf("Started IOC.\n")
optimopts = optimoptions(...
    @ga, ...
    'CreationFcn', 'gacreationlinearfeasible', ...
    'Display', 'iter', ...
    'FunctionTolerance', 2.6e-3, ...
    'PopulationSize', 25, ...
    'PlotFcn', 'gaplotbestf' ...
);
bllioc = bllioc.runConstantWeightGlobalIOC(optimopts);
fprintf("Ended IOC.\n");

save("run_IOC_globalConstantWeight.mat", "bllioc");