clear all;
close all;
clc;

Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 100;

doc = BoxLiftingDOC(splineDegree, splineNumKnots, splineEvaluationNumber);
% Constraints
doc = doc.addInitialJointConstraints();
doc = doc.addFinalCartesianConstraints();
doc = doc.addJointLimitConstraints();
doc = doc.addTorqueLimitConstraints();
doc = doc.addCopLimitConstraints();
doc = doc.addCollisionConstraints();
% Costs
nCosts = 7;
mCostPartition = 7;
doc = doc.addSumSquaredJointVelocitiesCost();
doc = doc.addSumSquaredJointAccelerationsCost();
doc = doc.addSumSquaredJointJerksCost();
doc = doc.addSumSquaredWristVelocityCost();
doc = doc.addSumSquaredWristAccelerationCost();
doc = doc.addSumSquaredJointTorquesCost();
doc = doc.addSumSquaredJointPowersCost();
% Create parameterized cost function
doc = doc.setParametrizedCostFunction();

% Set optimization options
sol_opt = struct;
sol_opt.ipopt.print_level = 0;
sol_opt.print_time = 0;
sol_opt.verbose = 0;
sol_opt.ipopt.sb = 'yes';
sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
sol_opt.regularity_check = true;

% Outputs
q_doc_all = zeros(6, splineEvaluationNumber, length(Trials));

tic
fprintf("%03d/%03d.", 0, length(Trials));
for TrialIndex = 1 : length(Trials)
    fprintf("\b\b\b\b\b\b\b\b%03d/%03d.", TrialIndex, length(Trials));
    % Extract current trial
    Trial = Trials(TrialIndex);
    
    % Instantiate trajectory, model and environment
    Trial.splineTrajectory = Trial.splineTrajectory.computeValuesAndStore(linspace(0, Trial.splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
    doc = doc.instantiateParameters(Trial.humanModel, Trial.liftingEnvironment, Trial.splineTrajectory);

    % Set solver for DOC along with options
    doc.opti.solver('ipopt', sol_opt);

    % Set the cost function parameters
    omega = zeros(nCosts, 1);
    omega(3) = 1;

    doc.setCostFunctionParameters(omega);

    % Solve doc
    sol_doc = doc.opti.solve();

    % Extract q
    q_doc_all(:, :, TrialIndex) = sol_doc.value(doc.q);
end
opt_time = toc;

save("test_BoxLiftingDOC_cf3_all_trials.mat", "q_doc_all", "opt_time");