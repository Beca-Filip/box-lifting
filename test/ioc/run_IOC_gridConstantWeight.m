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
doc = doc.addSumSquaredJointVelocitiesCost();
doc = doc.addSumSquaredJointAccelerationsCost();
doc = doc.addSumSquaredJointJerksCost();
doc = doc.addSumSquaredWristVelocityCost();
doc = doc.addSumSquaredWristAccelerationCost();
doc = doc.addSumSquaredJointTorquesCost();
doc = doc.addSumSquaredJointPowersCost();
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
bllioc = bllioc.runConstantWeightGridIOC(length(doc.costFunctionVector));
fprintf("Ended IOC.\n");

save("run_IOC_gridConstantWeight.mat", "bllioc");