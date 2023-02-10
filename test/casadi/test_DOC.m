%TEST_DOC is a script that tests a primitive DOC.
clear all
close all
clc

% Create object
opti = casadi.Opti();

%% Get CASADI Human model
% Create symbolic model
casadiHumanModel = HumanModel6DOF();
[casadiHumanModel, opti] = casadiHumanModel.defaultCasadiOptiInitialize(opti);

%% Get CASADI trajectory
% Spline trajectory parameters
dimension = 6;
degree = 5;
knotNumber = 10;
evaluationNumber = 50;
[casadiTrajectory, opti] = SplineTrajectory.defaultCasadiOptiInitialize(opti, dimension, degree, knotNumber, evaluationNumber);

%% Get numerical objects
% Human model
numHumanModel = HumanModel6DOF();
numHumanModel = numHumanModel.setDefaultKinematicPointsOfInterest();

% Select a target location
numQi = 0;
numQf = pi/6;
numQiQf = repmat(linspace(numQi, numQf, 2), [6, 1]);
numFkm = numHumanModel.forwardKinematicModel(numQiQf);

initialLocation = numFkm{7}(1:2, 1)
targetLocation = numFkm{7}(1:2, 2)

% Spline parameters
numKnotTimes = linspace(0, 5, knotNumber);
numKnotValues = repmat(linspace(numQi, numQf, knotNumber), [6, 1]);
numBoundaryConditions = SplineTrajectory.getDefaultBoundaryConditions(dimension, degree, knotNumber);
numComputationTimes = linspace(0, 5, evaluationNumber);

% Get the spline
numTrajectory = SplineTrajectory(numKnotTimes, numKnotValues, degree, numBoundaryConditions);
numTrajectory = numTrajectory.computeValuesAndStore(numComputationTimes, degree);

% Initial trajectory
q_init = numTrajectory.currentEvaluatedValuesAndDerivatives{1};
dq_init = numTrajectory.currentEvaluatedValuesAndDerivatives{2};
ddq_init = numTrajectory.currentEvaluatedValuesAndDerivatives{3};
PTS_init = numHumanModel.forwardKinematicModel(q_init);
V_init = numHumanModel.forwardKinematicVelocityModel(q_init, dq_init);
A_init = numHumanModel.forwardKinematicAccelerationModel(q_init, dq_init, ddq_init);


% Plot initial joint trajectory
figure;
hold on;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"angle [rad]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, q_init,...
                                  [], opts, plot_shape);
plot_vector_quantities_opts_shape(numComputationTimes([1, end]), repmat(numHumanModel.LowerJointLimits, [1, 2]),...
                                  [], [], plot_shape, 'r--');
plot_vector_quantities_opts_shape(numComputationTimes([1, end]), repmat(numHumanModel.UpperJointLimits, [1, 2]),...
                                  [], [], plot_shape, 'r--');

% Plot initial joint velocities
figure;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"ang. vel. [rad.s$^{-1}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, dq_init,...
                                  [], opts, plot_shape);
                              
% Plot initial joint accelerations
figure;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"ang. acc. [rad.s$^{-2}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, ddq_init,...
                                  [], opts, plot_shape);

% Plot initial Cartesian trajectory
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
X_init = zeros(nFrames, evaluationNumber);
Y_init = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    X_init(currFrame, :) = PTS_init{currFrame}(1, :);
    Y_init(currFrame, :) = PTS_init{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"position [m]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [X_init; Y_init],...
                                  [], opts, plot_shape);

% Plot initial Cartesian velocities
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
VX_init = zeros(nFrames, evaluationNumber);
VY_init = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    VX_init(currFrame, :) = V_init{currFrame}(1, :);
    VY_init(currFrame, :) = V_init{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"velocity [m.s$^{-1}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [VX_init; VY_init],...
                                  [], opts, plot_shape);
                              
% Plot initial Cartesian accelerations
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
AX_init = zeros(nFrames, evaluationNumber);
AY_init = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    AX_init(currFrame, :) = A_init{currFrame}(1, :);
    AY_init(currFrame, :) = A_init{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"acceleration [m.s$^{-2}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [AX_init; AY_init],...
                                  [], opts, plot_shape);
                              
% Animate initial trajectory
figure;
hold on;
plot(initialLocation(1), initialLocation(2), 'rx', "DisplayName", '$p_i$');
plot(targetLocation(1), targetLocation(2), 'rx', "DisplayName", '$p_f$');
Animate_nDOF(numTrajectory.currentEvaluatedValuesAndDerivatives{1}, numHumanModel.L, round(5/evaluationNumber, 2))
pause
%% For the cost and constraints
% Get trajectory and derivatives
q = casadiTrajectory.currentEvaluatedValuesAndDerivatives{1};
dq = casadiTrajectory.currentEvaluatedValuesAndDerivatives{2};
ddq = casadiTrajectory.currentEvaluatedValuesAndDerivatives{3};

% Null external forces
fFOOT = zeros(6, evaluationNumber);
fHAND = zeros(6, evaluationNumber);

% Get dynamics
[tau, f_grf] = casadiHumanModel.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND);

% Calculate the FKM
PTS = casadiHumanModel.forwardKinematicModel(q);
V = casadiHumanModel.forwardKinematicVelocityModel(q, dq);
A = casadiHumanModel.forwardKinematicAccelerationModel(q, dq, ddq);

% Formulate cost
% costf = sum(sum(tau.^2)) / 400;
% costf = sum(sum(V{7}.^2));
% costf = sum(sum(V{8}.^2 + V{7}.^2 + V{6}.^2 + V{5}.^2 + V{4}.^2 + V{3}.^2 + V{2}.^2));
% costf = sum(sum(V{8}.^2 + V{7}.^2 + V{6}.^2 + V{5}.^2 + V{4}.^2 + V{3}.^2 + V{2}.^2)) + sum(sum(tau.^2)) / 400;
% costf = sum(sum(dq.^2));
costf = 30*sum(sum(V{7}.^2)) + sum(sum(dq.^2));
% Formulate equality constraints
h1 = initialLocation - PTS{7}(1:2, 1);
h2 = targetLocation - PTS{7}(1:2, end);
g1 = (-q + repmat(casadiHumanModel.LowerJointLimits, 1, evaluationNumber));
g2 = ( q - repmat(casadiHumanModel.UpperJointLimits, 1, evaluationNumber));

% Add to optimizer
opti.minimize(costf);
ceq1 = (h1 == 0);
ceq2 = (h2 == 0);
c1 = (g1(:) <= 0);
c2 = (g2(:) <= 0);
opti.subject_to(ceq1);
opti.subject_to(ceq2);
opti.subject_to(c1);
opti.subject_to(c2);

% Instantiate
[casadiHumanModel, opti] = casadiHumanModel.casadiInstantiateOptiParameters(numHumanModel, opti);
[casadiTrajectory, opti] = casadiTrajectory.casadiInstantiateOptiParameters(numTrajectory, opti);

%%
% Choose solver with options
sol_opt= struct;
% sol_opt.ipopt.print_level = 0;
% sol_opt.print_time =0;
% sol_opt.verbose = 0;
% sol_opt.ipopt.sb ='yes';
sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
sol_opt.regularity_check = true;
opti.solver('ipopt', sol_opt);

% Solve once
sol1 = opti.solve();
% disp(sol1.stats.iter_count);

% % New options
% sol_opt= struct;
% sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
% sol_opt.regularity_check = true;
% opti.solver('ipopt', sol_opt);
% % Warm start
% opti.set_initial(sol1.value_variables())
% % Solve twice
% sol2 = opti.solve();
% disp(sol2.stats.iter_count);

%%
% Retrieve solution
solKnotValues = sol1.value(casadiTrajectory.knotValues);

% Solution trajectory
solTrajectory = SplineTrajectory(numKnotTimes, solKnotValues, degree, numBoundaryConditions);
solTrajectory = solTrajectory.computeValuesAndStore(numComputationTimes, degree);

% Solution trajectory
q_sol = solTrajectory.currentEvaluatedValuesAndDerivatives{1};
dq_sol = solTrajectory.currentEvaluatedValuesAndDerivatives{2};
ddq_sol = solTrajectory.currentEvaluatedValuesAndDerivatives{3};
PTS_sol = numHumanModel.forwardKinematicModel(q_init);
V_sol = numHumanModel.forwardKinematicVelocityModel(q_init, dq_init);
A_sol = numHumanModel.forwardKinematicAccelerationModel(q_init, dq_init, ddq_init);

% Plot final joint trajectory
figure;
hold on;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"angle [rad]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, q_sol,...
                                  [], opts, plot_shape);
plot_vector_quantities_opts_shape(numComputationTimes([1, end]), repmat(numHumanModel.LowerJointLimits, [1, 2]),...
                                  [], [], plot_shape, 'r--');
plot_vector_quantities_opts_shape(numComputationTimes([1, end]), repmat(numHumanModel.UpperJointLimits, [1, 2]),...
                                  [], [], plot_shape, 'r--');

% Plot final joint velocities
figure;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"ang. vel. [rad.s$^{-1}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, dq_sol,...
                                  [], opts, plot_shape);
                              
% Plot final joint accelerations
figure;
plot_shape = [2, 3];
joint_names = ["Ankle joint", "Knee joint", "Hip joint", "Back joint", "Shoulder joint", "Elbow joint"];
opts.title = @(n) {sprintf("%s", joint_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"ang. acc. [rad.s$^{-2}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, ddq_sol,...
                                  [], opts, plot_shape);
                              
% Plot final Cartesian trajectory
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
X_sol = zeros(nFrames, evaluationNumber);
Y_sol = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    X_sol(currFrame, :) = PTS_sol{currFrame}(1, :);
    Y_sol(currFrame, :) = PTS_sol{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"position [m]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [X_sol; Y_sol],...
                                  [], opts, plot_shape);

% Plot initial Cartesian velocities
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
VX_sol = zeros(nFrames, evaluationNumber);
VY_sol = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    VX_sol(currFrame, :) = V_sol{currFrame}(1, :);
    VY_sol(currFrame, :) = V_sol{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"velocity [m.s$^{-1}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [VX_sol; VY_sol],...
                                  [], opts, plot_shape);
                              
% Plot initial Cartesian accelerations
figure;
nFrames = length(numHumanModel.KPOI);
plot_shape = [2, nFrames];
frame_names = [arrayfun(@(n) sprintf("%s X-axis", numHumanModel.KPOI(n).Name), 1:nFrames), ...
               arrayfun(@(n) sprintf("%s Y-axis", numHumanModel.KPOI(n).Name), 1:nFrames)];
AX_sol = zeros(nFrames, evaluationNumber);
AY_sol = zeros(nFrames, evaluationNumber);
for currFrame = 1 : nFrames
    AX_sol(currFrame, :) = A_sol{currFrame}(1, :);
    AY_sol(currFrame, :) = A_sol{currFrame}(2, :);
end
opts.title = @(n) {sprintf("%s", frame_names(n)), 'interpreter', 'latex'};
opts.xlabel = @(n) {"time [s]", 'interpreter', 'latex'};
opts.ylabel = @(n) {"acceleration [m.s$^{-2}$]", 'interpreter', 'latex'};
plot_vector_quantities_opts_shape(numComputationTimes, [AX_sol; AY_sol],...
                                  [], opts, plot_shape);

%% Animate
figure;
hold on;
plot(initialLocation(1), initialLocation(2), 'rx', "DisplayName", '$p_i$');
plot(targetLocation(1), targetLocation(2), 'rx', "DisplayName", '$p_f$');
Animate_nDOF(solTrajectory.currentEvaluatedValuesAndDerivatives{1}, numHumanModel.L, round(5/evaluationNumber, 2))