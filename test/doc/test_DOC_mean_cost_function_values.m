clear all;
close all;
clc;

Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 102;

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

% Set optimization options
sol_opt = struct;
% sol_opt.ipopt.print_level = 0;
% sol_opt.print_time = 0;
% sol_opt.verbose = 0;
sol_opt.ipopt.sb = 'yes';
sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
sol_opt.regularity_check = true;

% Set solver for DOC along with options
doc.opti.solver('ipopt', sol_opt);

% Create parameter vector
omega = ones(7, splineEvaluationNumber);
% Vector of infeasible initial DOCs
infeas_initial = [];

% CF vector
cfVector = zeros(length(doc.costFunctionVector), length(Trials));

fprintf("DOC mean value of costs.\n");
% For every trial
for TrialIndex = 1 : length(Trials)
    % Interpolate the trial with the right number of points
    Trials(TrialIndex).splineTrajectory = Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
    % Instantiate the DOC
    doc = doc.instantiateParameters(Trials(TrialIndex).humanModel, Trials(TrialIndex).liftingEnvironment, Trials(TrialIndex).splineTrajectory);
   
    % Get CF vector
    cfVector(:, TrialIndex) = doc.getInitial(doc.costFunctionVector);
    
end
%%
fprintf("mean \t std\n");
[mean(cfVector, 2), std(cfVector, [], 2)]
fprintf("mean - std \t mean + std\n")
[mean(cfVector, 2) - std(cfVector, [], 2), mean(cfVector, 2) + std(cfVector, [], 2)]
