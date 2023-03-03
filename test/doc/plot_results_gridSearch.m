Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");
TrialIndex = 1;
[~, indmin] = min(mean(RMSE));

q_m = q_arr(:, :, indmin);

time_vec = linspace(0, Trials(TrialIndex).splineTrajectory.duration, size(q_m , 2));
q_e = Trials(TrialIndex).splineTrajectory.computeValues(time_vec, 4);

q_h = q_e{1};

figure;
names = ["Ankle", "Knee", "Hip", "Back", "Shoulder", "Elbow"];
plotopts = [];
plotopts.title = @(n) {sprintf("%s joint anles.", names(n)), "interpreter", "latex"};
plotopts.xlabel = @(n) {"time [s]", "interpreter", "latex"};
plotopts.ylabel = @(n) {sprintf("$q_%d$ [deg]", n), "interpreter", "latex"};
hold on;
plot_vector_quantities_opts_shape(time_vec, rad2deg(q_m), [], plotopts, [2,3], 'DisplayName', '$q_{opt}$');
plot_vector_quantities_opts_shape(time_vec, rad2deg(q_h), [], [], [2,3], 'DisplayName', '$q_{hum}$');
legend("interpreter", "latex");

pause;

figure;
subplot(1, 2, 1)
animopts = [];
animopts.title = "Opt";
Animate_nDOF(q_m, Trials(TrialIndex).humanModel.L, round(3*224/100 * 0.01, 2), animopts);
subplot(1, 2, 2)
animopts.title = "Human";
Animate_nDOF(q_h, Trials(TrialIndex).humanModel.L, round(3*224/100 * 0.01, 2), animopts);

pause;

%%
figure;
Animate_Two_nDOF(q_h, Trials(TrialIndex).humanModel.L, q_m, Trials(TrialIndex).humanModel.L, round(3*224/100 * 0.01, 2));