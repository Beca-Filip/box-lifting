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

% Instantiate trajectory, model and environment
TrialIndex = 1;
Trials(TrialIndex).splineTrajectory = Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
doc = doc.instantiateParameters(Trials(TrialIndex).humanModel, Trials(TrialIndex).liftingEnvironment, Trials(TrialIndex).splineTrajectory);

% Set solver for DOC along with options
doc.opti.solver('ipopt', sol_opt);


% Get parameter array
omega_arr = prob_simplex_ndim(nCosts-1, mCostPartition);
% Get RMSE array
RMSE = zeros(6, size(omega_arr, 1));
% Get joint angles array
q_arr = zeros([size(Trials(TrialIndex).splineTrajectory.currentEvaluatedValuesAndDerivatives{1}), size(omega_arr, 1)]);

tic
fprintf("Iteration %05d/%05d.", 0, size(omega_arr, 1));
% For every parameter
for ii = 1 : size(omega_arr, 1)
    
    fprintf("\b\b\b\b\b\b\b\b\b\b\b\b%05d/%05d.", ii, size(omega_arr, 1));
   
    % Extract the cost function parameters
    omega = omega_arr(ii, :).';
    % Set the cost function parameters
    doc = doc.setCostFunctionParameters(omega);
    
    % Solve doc
    sol_doc = doc.opti.solve();
    
    % Extract 
    q_arr(:, :, ii) = sol_doc.value(doc.q);
    
    % Caclulate RMSE
    RMSE(:, ii) = row_rmse(Trials(TrialIndex).splineTrajectory.currentEvaluatedValuesAndDerivatives{1}, q_arr(:, :, ii));
end
fprintf("\n");
toc
save("test_BoxLiftingDOC_gridSearch.mat", "boxliftingdoc", "q", "RMSE", "omega_arr");

