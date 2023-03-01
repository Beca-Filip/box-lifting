clear all;
close all;
clc;

Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

splineDegree = 5;
splineNumKnots = 10;
splineEvaluationNumber = 100;

doc = BoxLiftingDOC(splineDegree, splineNumKnots, splineEvaluationNumber, "variable_final_velocities");
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
% Set parameters
% doc = doc.setCostFunctionParameters(ones(7, 1));
omega0 = zeros(7, 1);
omega0(3) = 1;
omega0(4) = 1;
doc = doc.setCostFunctionParameters(omega0);

sol_opt = struct;
% sol_opt.ipopt.print_level = 0;
% sol_opt.print_time =0;
% sol_opt.verbose = 0;
% sol_opt.ipopt.sb ='yes';
sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
sol_opt.regularity_check = true;

TrialIndex = 1;
Trials(TrialIndex).splineTrajectory = Trials(TrialIndex).splineTrajectory.computeValuesAndStore(linspace(0, Trials(TrialIndex).splineTrajectory.duration, splineEvaluationNumber), splineDegree-1);
doc = doc.instantiateParameters(Trials(TrialIndex).humanModel, Trials(TrialIndex).liftingEnvironment, Trials(TrialIndex).splineTrajectory);

doc.opti.solver('ipopt', sol_opt);
%%
sol_doc = doc.opti.solve();

%%
figure;
q = sol_doc.value(doc.q);
L = sol_doc.value(doc.casadiHumanModel.L);
Ts = sol_doc.value(doc.casadiSplineTrajectory.duration ./ numel(doc.casadiSplineTrajectory.currentEvaluatedTimes));

Animate_nDOF(q, L, round(Ts*3, 2));

%% 
time_vec = sol_doc.value(doc.casadiSplineTrajectory.currentEvaluatedTimes);
dq = sol_doc.value(doc.dq);
v_wri = sol_doc.value(doc.v_wri);

figure;
names = ["Ankle", "Knee", "Hip", "Back", "Shoulder", "Elbow"];
plotopts = [];
plotopts.title = @(n) {sprintf("%s joint velocity.", names(n)), "interpreter", "latex"};
plotopts.xlabel = @(n) {"time [s]", "interpreter", "latex"};
plotopts.ylabel = @(n) {sprintf("$\\frac{\\partial q_%d}{\\partial t}$ [rad.s$^{-1}$]", n), "interpreter", "latex"};
plot_vector_quantities_opts_shape(time_vec, dq, [], plotopts, [2, 3], 'r');

figure;
names = ["X", "Y"];
plotopts = [];
plotopts.title = @(n) {sprintf("Wrist %s velocity.", names(n)), "interpreter", "latex"};
plotopts.xlabel = @(n) {"time [s]", "interpreter", "latex"};
plotopts.ylabel = @(n) {sprintf("$v_%s$ [m.s$^{-1}$]", names(n)), "interpreter", "latex"};
plot_vector_quantities_opts_shape(time_vec, v_wri, [], plotopts, [2, 1], 'r');