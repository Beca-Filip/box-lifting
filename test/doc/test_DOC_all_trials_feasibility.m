clear all;
close all;
clc;

Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 102;

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
doc = doc.addSumSquaredJointJerksCost();
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
omega = ones(2, splineEvaluationNumber);
omega(1, :) = 1e-3;
omega(2, 1:round(end/3)) = 10^(1.7);
omega(2, round(2*end/3)+1:end) = 10^(1.7);
% Set parameter vector
doc = doc.setCostFunctionParameters(omega);

% Vector of infeasible initial DOCs
infeas_initial = [];

fprintf("DOC Feasibility Checker.\n");
% For every trial
for TrialIndex = 1 : length(Trials)
    fprintf("BEGIN DOC Feasibility Check %03d/%03d.\n\n\n", TrialIndex, length(Trials));
    % Interpolate the trial with the right number of points
    Trials(TrialIndex).splineTrajectory = Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
    % Instantiate the DOC
    doc = doc.instantiateParameters(Trials(TrialIndex).humanModel, Trials(TrialIndex).liftingEnvironment, Trials(TrialIndex).splineTrajectory);
   
    % Check feasibilility
    ErrorCounter = doc.debugInitialDOC();
    % If ErrorCounter is bigger than 0
    if ErrorCounter > 0
        infeas_initial = [infeas_initial, TrialIndex];
    end
    
    fprintf("\n\n\nEND DOC Feasibility Check %03d/%03d.\n\n", TrialIndex, length(Trials));
end

