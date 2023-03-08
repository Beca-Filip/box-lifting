clear all;
close all;
clc;

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
nCosts = 7;
mCostPartition = 7;
doc = doc.addSumSquaredJointJerksCost();
doc = doc.addSumSquaredWristAccelerationCost();
doc = doc.addSumSquaredJointTorquesCost();
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

% Get ndimensional indices
nT = 3;
ncf = 3;
vals = [1/3, 10/3, 100/3];
listind = list_ndim(nT*ncf, 3 * ones(1, nT*ncf));
% Get parameter array
omega_arr = reshape(listind.', [ncf, nT, size(listind, 1)]);
omega_arr(omega_arr == 1) = vals(1);
omega_arr(omega_arr == 2) = vals(2);
omega_arr(omega_arr == 3) = vals(3);

% Get RMSE array
RMSE = zeros(6, size(omega_arr, 3));
% Get joint angles array
q_arr = zeros([size(Trials(TrialIndex).splineTrajectory.currentEvaluatedValuesAndDerivatives{1}), size(omega_arr, 3)]);

tic
fprintf("Iteration %05d/%05d.", 0, size(omega_arr, 3));
% For every parameter
for ii = 1 : size(omega_arr, 3)
    
    fprintf("\b\b\b\b\b\b\b\b\b\b\b\b%05d/%05d.", ii, size(omega_arr, 3));
   
    % Extract the cost function parameters
    omega = omega_arr(:, :, ii).';
    
    indx1 = 1 : round(splineEvaluationNumber/3);
    indx2 = round(splineEvaluationNumber/3) + 1 : round(2*splineEvaluationNumber/3);
    indx3 = round(2*splineEvaluationNumber/3) + 1 : splineEvaluationNumber;
    omega_t(:, indx1) = repmat(omega(:, 1), [1, length(indx1)]);
    omega_t(:, indx2) = repmat(omega(:, 2), [1, length(indx2)]);
    omega_t(:, indx3) = repmat(omega(:, 3), [1, length(indx3)]);
    
    % Set the cost function parameters
    doc = doc.setCostFunctionParameters(omega_t);
    
    % Solve doc
    sol_doc = doc.opti.solve();
    
    % Extract 
    q_arr(:, :, ii) = sol_doc.value(doc.q);
    
    % Caclulate RMSE
    RMSE(:, ii) = row_rmse(Trials(TrialIndex).splineTrajectory.currentEvaluatedValuesAndDerivatives{1}, q_arr(:, :, ii));
end
fprintf("\n");
toc
%%
save("test_BoxLiftingDOC_tvw_gridSearch.mat", "doc", "q_arr", "RMSE", "omega_arr");

