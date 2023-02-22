%STEP2 performs the dynamic identification on the calibration data.
close all;
clc;

%% Define constants
% Number of subjects
NO_SUBJECTS = 7;

% Define the rotation matrix from the Markers frame to the Human frame
base_R_markers = roty(180);
markers_R_base = base_R_markers.';

% Define the rotation matrix from the Forceplate frame to the Markers frame
markers_R_fp = rotx(90);
fp_R_markers = markers_R_fp.';

% Define the rotation matrix from the Forceplate to the Human base frame
base_R_fp = base_R_markers * markers_R_fp;
fp_R_base = base_R_fp.';

% Define the supposed vector from markers to forceplate frame in
% markers frame ( Center of Forceplate )
markers_r_markers_fp = [0.2; 0; 0.3];

% Calibration data filepath
kinematic_identification_filepath = "../processed_data/Calibration/kinematically_calibrated.mat";
Calibration = importdata(kinematic_identification_filepath, "Calibration");

% Output folder path
common_output_folderpath = "../processed_data/Calibration";

% Output filename
output_filename = "dynamically_and_kinematically_calibrated";

%% Perform the computations

% Prealocate an array of calibrated human models
CalibratedHumanModels = [];

% For each subject
for numSubj = 2 : NO_SUBJECTS
    
    % Subject name
    subj = sprintf("S%d", numSubj);
    
    % Output folderpath
    subject_output_folderpath = sprintf("%s/%s", common_output_folderpath, subj);
    if ~exist(subject_output_folderpath, 'dir')
        mkdir(subject_output_folderpath);
    end
    
    % Get the kinematically calibrated human model from kinematic identification
    kinCalibratedHumanModel = Calibration.(subj).humanModel;        
    
    % Get number of samples
    nbSamples = Calibration.(subj).NumberSamples;
%     nbSamples = 8000;
    
    % Make a time vector rate
    SamplingTime = Calibration.(subj).SamplingTime;
    SamplingFrequency = Calibration.(subj).SamplingFrequency;
    time_vec = 0 : SamplingTime : (nbSamples-1)*SamplingTime;
    
    % Define filtering parameters for raw data
    filt_fs = SamplingFrequency;
    filt_cutoff = 2;
    filt_order = 5;
    
    % Prepare the markers structure
    % Filter the markers
    Markers = MarkersFilter(Calibration.(subj).Markers, filt_fs, filt_cutoff, filt_order);
    % Extract position vector of the HumanModel6DOF base frame in the OptiTrack frame
    markers_r_markers_base = mean(Markers.BODY.RANK).';
    % Translate and rotate markers
    Markers = MarkersTranslateRotate(-markers_r_markers_base, base_R_markers, Markers);
    
    % Extract heel and toe position
    HeelPosition = mean(Markers.BODY.RHEE - Markers.BODY.RANK).';
    ToePosition = mean(Markers.BODY.RTOE - Markers.BODY.RANK).';
    kinCalibratedHumanModel.HeelPosition = HeelPosition(1:2);
    kinCalibratedHumanModel.ToePosition = ToePosition(1:2);
    
    % Prepare the angles, velocities and accelerations
    % Filter q, and find dq and ddq
    q_filt = lowpass_filter(Calibration.(subj).kinematicIdentification.q(:, 1:nbSamples), filt_fs, filt_cutoff, filt_order);
    dq_filt = diff(q_filt, 1, 2) ./ SamplingTime;
    ddq_filt = diff(q_filt, 2, 2) ./ SamplingTime^2;
    dq_filt = [dq_filt, dq_filt(:, end)];
    ddq_filt = [ddq_filt, ddq_filt(:, end-1:end)];
    
    % Calculate the external wrenches by the model
    % Define external wrenches (there are none)
    fFOOT = zeros(6, nbSamples);
    fHAND = zeros(6, nbSamples);
    % Calculate the ground reaction wrenches
    [tau, base_grf_model] = kinCalibratedHumanModel.inverseDynamicModel(q_filt, dq_filt, ddq_filt, fFOOT, fHAND);
    base_cop_model = base_grf_model(6, :) ./ base_grf_model(2, :);
        
    % Prepare the forceplate data
    % Lowpass filter the forceplate data
    fp_Forceplate_filt = ForceplateFilter(Calibration.(subj).Forceplate, filt_fs, filt_cutoff, filt_order);
    % Index the forceplate data
    fp_Forceplate_filt = ForceplateIndex(fp_Forceplate_filt, 1:nbSamples);
    
    % Extract in grf and cop form
    fp_grf_fp = [fp_Forceplate_filt.Forces.'; fp_Forceplate_filt.Moments.'];
    fp_cop_fp = fp_Forceplate_filt.COP(:, 1).';
    
    % Transport model wrenches to global/markers frame
    markers_grf_model = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model);
    markers_cop_model = markers_grf_model(6, :) ./ markers_grf_model(2, :);
    % Transport forceplate wrenches to global/markers frame
    markers_grf_fp = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp);
    markers_cop_fp = markers_grf_fp(6, :) ./ markers_grf_fp(2, :);
    
    % Plot predictions
    fig_dyn_predictions = figure('Position', [100, 50, 1280, 720]);
    names = ["$F_x$", "$F_y$", "$M_z$", '${\rm COP}_x$'];
    units = ["N", "N", "N.m", "m"];
    plotopts = [];
    plotopts.title = @(n) {names(n), 'interpreter', 'latex', 'fontsize', 13};
    plotopts.xlim = @(n){[time_vec(1), time_vec(end)]};
    plotopts.xlabel = @(n) {"$t$[s]", 'interpreter', 'latex', 'fontsize', 13};
    plotopts.ylabel = @(n) {sprintf("%s [%s]", names(n), units(n)), 'interpreter', 'latex', 'fontsize', 13};
    plot_vector_quantities_opts_shape(time_vec, [markers_grf_model([1,2,6], :); markers_cop_model], [], plotopts, [2, 2], 'c', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm AT Model}$');
    plot_vector_quantities_opts_shape(time_vec, [markers_grf_fp([1,2,6], :); markers_cop_fp], [], [], [2, 2], 'b', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm Forceplate}$');
    legend('interpreter', 'latex', 'location', 'best');
    
%     %% Do the dynamic identification 
%     dimModel = DynamicIdentificationHumanModel6DOF(nbSamples);
%     % Initialize the search
%     dimModel = dimModel.instantiateParameters(fp_Forceplate_filt, q_filt, dq_filt, ddq_filt, kinCalibratedHumanModel, markers_R_base, markers_R_fp, markers_r_markers_base, markers_r_markers_fp);
%     % Solver choice
%     dimModel.opti.solver('ipopt');
%     % Solution
%     dim_sol = dimModel.opti.solve();
%     
    %% Do the dynamic identification 
    % Define how many samples to remove from beginning and end of
    % forceplate traj because of the artefacts (forces = 0 for first and
    % last 50 ms)
    nRmvSmpl = 10;
    % Remove the samples from forceplate and velocities
    fp_Forceplate_filt_rmsmp = ForceplateIndex(fp_Forceplate_filt, nRmvSmpl + 1 : nbSamples - nRmvSmpl);
    q_filt = q_filt(:, nRmvSmpl + 1 : nbSamples - nRmvSmpl);
    dq_filt = dq_filt(:, nRmvSmpl + 1 : nbSamples - nRmvSmpl);
    ddq_filt = ddq_filt(:, nRmvSmpl + 1 : nbSamples - nRmvSmpl);    
    
    % Create the model
    dimModel = DynamicIdentificationHumanModel6DOF(nbSamples - 2 * nRmvSmpl);
    % Initialize the search
    dimModel = dimModel.instantiateParameters(fp_Forceplate_filt_rmsmp, q_filt, dq_filt, ddq_filt, kinCalibratedHumanModel, markers_R_base, markers_R_fp, markers_r_markers_base, markers_r_markers_fp);
    % Solver choice
    dimModel.opti.solver('ipopt');
    % Solution
    dim_sol = dimModel.opti.solve();
    %% Get the ground reaction forces and cop
    base_grf_model_dim = dimModel.computeBaseGRFModel(dim_sol);    
    base_cop_model_dim = base_grf_model_dim(6, :) ./ base_grf_model_dim(2, :);

    [markers_grf_model_dim, markers_grf_fp_dim] = dimModel.computeMarkersGRFModelAndForceplate(dim_sol);
    markers_cop_model_dim = markers_grf_model_dim(6, :) ./ markers_grf_model_dim(2, :);
    markers_cop_fp_dim = markers_grf_fp_dim(6, :) ./ markers_grf_fp_dim(2, :);
    
    % Plot predictions
    fig_dyn_predictions_dim = figure('Position', [100, 50, 1280, 720]);
    names = ["$F_x$", "$F_y$", "$M_z$", '${\rm COP}_x$'];
    units = ["N", "N", "N.m", "m"];
    plotopts = [];
    plotopts.title = @(n) {names(n), 'interpreter', 'latex', 'fontsize', 13};
    plotopts.xlim = @(n){[time_vec(1), time_vec(end)]};
    plotopts.xlabel = @(n) {"$t$[s]", 'interpreter', 'latex', 'fontsize', 13};
    plotopts.ylabel = @(n) {sprintf("%s [%s]", names(n), units(n)), 'interpreter', 'latex', 'fontsize', 13};
    plot_vector_quantities_opts_shape(time_vec(nRmvSmpl+1 : nbSamples - nRmvSmpl), [markers_grf_model_dim([1,2,6], :); markers_cop_model_dim], [], plotopts, [2, 2], 'c', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm AT Model}$');
    plot_vector_quantities_opts_shape(time_vec(nRmvSmpl+1 : nbSamples - nRmvSmpl), [markers_grf_fp_dim([1,2,6], :); markers_cop_fp_dim], [], [], [2, 2], 'b', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm Forceplate}$');
    legend('interpreter', 'latex', 'location', 'best');
    
    %% Display and Plot the residual of the GRFs and COPs and
    % Residuals of the anthropometric tables model
    res_fx = markers_grf_model(1, :).' - markers_grf_fp(1, :).';
    res_fy = markers_grf_model(2, :).' - markers_grf_fp(2, :).';
    res_mz = markers_grf_model(6, :).' - markers_grf_fp(6, :).';
    res_cop = markers_cop_model.' - markers_cop_fp.';
    fprintf("sqrt(sum(sum(res_fx.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(res_fx.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(res_fy.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(res_fy.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(res_mz.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(res_mz.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(res_cop.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(res_cop.^2) ./ nbSamples)));
    % Residual of the dynamically identified model
    sol_resnorms = dimModel.computeResidualNorms(dim_sol);
    sol_res_fx = sol_resnorms(:, 1);
    sol_res_fy = sol_resnorms(:, 2);
    sol_res_mz = sol_resnorms(:, 3);
    sol_res_cop = sol_resnorms(:, 4);
    fprintf("sqrt(sum(sum(sol_res_fx.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(sol_res_fx.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(sol_res_fy.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(sol_res_fy.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(sol_res_mz.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(sol_res_mz.^2) ./ nbSamples)));
    fprintf("sqrt(sum(sum(sol_res_cop.^2) ./ nbSamples)) = %.4f\n", ...
             sqrt(sum(sum(sol_res_cop.^2) ./ nbSamples)));
         
    % Store the RMSE
    rmse_fx = sqrt(sum(sum(sol_res_fx.^2) ./ nbSamples));
    rmse_fy = sqrt(sum(sum(sol_res_fy.^2) ./ nbSamples));
    rmse_mz = sqrt(sum(sum(sol_res_mz.^2) ./ nbSamples));
    rmse_cop = sqrt(sum(sum(sol_res_cop.^2) ./ nbSamples));
    
    wrenches_rmse = [rmse_fx; rmse_fy; rmse_mz; rmse_cop];
    
    %% Load the numeric model result
    % Load the human model parameters
    identifiedHumanModel = dimModel.computeNumericalModel(dim_sol);
    % Get the 
    markers_r_markers_fp_dim = dimModel.compute_markers_r_markers_fp(dim_sol);
    
    markers_r_markers_fp
    markers_r_markers_fp_dim
    
    %% Barplot the retrieved parameters
    
    % Plot the masses
    % Names of the body parts
    names = ["Feet", "Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms", "Hands", "Neck-Head"];
    % Data vector
    mass = [...
    [kinCalibratedHumanModel.MFOOT, kinCalibratedHumanModel.M, kinCalibratedHumanModel.MHAND, kinCalibratedHumanModel.MHEAD];
    [identifiedHumanModel.MFOOT, identifiedHumanModel.M, identifiedHumanModel.MHAND, identifiedHumanModel.MHEAD];
    ].';
    % Figure plot
    fig_M = figure('Position', [100, 50, 1280, 720]);
    massbar = bar(mass);
    % Aesthetics
    xticklabels(names);
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
    xlabel("Body parts", 'interpreter', 'latex', 'fontsize', 13);
    ylabel("Segment masses [kg]", 'interpreter', 'latex', 'fontsize', 13);
    legend("Anthropometric Tables", "Dynamic Identification", 'interpreter', 'latex', 'fontsize', 13);
    % Title
    titlestring = {sprintf("Subject %s segment masses:", subj)};
    title(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    
    % Plot the inertias
    % Names of the body parts
    names = ["Feet", "Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms", "Hands", "Neck-Head"];
    % Data vector
    inertias = [...
    [kinCalibratedHumanModel.IzzFOOT, kinCalibratedHumanModel.Izz, kinCalibratedHumanModel.IzzHAND, kinCalibratedHumanModel.IzzHEAD];
    [identifiedHumanModel.IzzFOOT, identifiedHumanModel.Izz, identifiedHumanModel.IzzHAND, identifiedHumanModel.IzzHEAD];
    ].';
    % Figure plot
    fig_Izz = figure('Position', [100, 50, 1280, 720]);
    inertiabar = bar(inertias);
    % Aesthetics
    xticklabels(names);
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
    xlabel("Body parts", 'interpreter', 'latex', 'fontsize', 13);
    ylabel("Segment inertias [kg.m$^2$]", 'interpreter', 'latex', 'fontsize', 13);
    legend("Anthropometric Tables", "Dynamic Identification", 'interpreter', 'latex', 'fontsize', 13);
    % Title
    titlestring = {sprintf("Subject %s segment inertias:", subj)};
    title(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    
    % Plot the COMs
    % Names of the body parts
    names = ["Feet", "Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms", "Hands", "Neck-Head"];
    % Data vector
    comsx = [...
    [kinCalibratedHumanModel.CoMFOOT(1), kinCalibratedHumanModel.CoM(1, :), kinCalibratedHumanModel.CoMHAND(1), kinCalibratedHumanModel.CoMHEAD(1)];
    [identifiedHumanModel.CoMFOOT(1), identifiedHumanModel.CoM(1, :), identifiedHumanModel.CoMHAND(1), identifiedHumanModel.CoMHEAD(1)];
    ].';
    comsy = [...
    [kinCalibratedHumanModel.CoMFOOT(2), kinCalibratedHumanModel.CoM(2, :), kinCalibratedHumanModel.CoMHAND(2), kinCalibratedHumanModel.CoMHEAD(2)];
    [identifiedHumanModel.CoMFOOT(2), identifiedHumanModel.CoM(2, :), identifiedHumanModel.CoMHAND(2), identifiedHumanModel.CoMHEAD(2)];
    ].';
    % Figure plot
    fig_CoM = figure('Position', [100, 50, 1280, 720]);
    % Subplot for X axis coms
    subplot(2, 1, 1);
    comsxbar = bar(comsx);
    % Aesthetics
    xticklabels(names);
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
    xlabel("Body parts", 'interpreter', 'latex', 'fontsize', 13);
    ylabel("Segment X-axis CoM positions [m]", 'interpreter', 'latex', 'fontsize', 13);
    % Title
    titlestring = {sprintf("Subject %s segment CoM positions along segment X-axis:", subj)};
    title(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    
    % Subplot for Y axis coms
    subplot(2, 1, 2);
    comsybar = bar(comsy);
    % Aesthetics
    xticklabels(names);
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
    xlabel("Body parts", 'interpreter', 'latex', 'fontsize', 13);
    ylabel("Segment Y-axis CoM positions [m]", 'interpreter', 'latex', 'fontsize', 13);
    legend("Anthropometric Tables", "Dynamic Identification", 'interpreter', 'latex', 'fontsize', 13);
    % Title
    titlestring = {sprintf("Subject %s segment CoM positions along segment Y-axis:", subj)};
    title(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    
    % Figure saving
    figname_dyn_predictions = sprintf("%s/GRF_model_vs_meas_anthropo_tab.fig", subject_output_folderpath);
    saveas(fig_dyn_predictions, figname_dyn_predictions);
    figname_dyn_predictions_dim = sprintf("%s/GRF_model_vs_meas_DIM.fig", subject_output_folderpath);
    saveas(fig_dyn_predictions_dim, figname_dyn_predictions_dim);
    figname_M = sprintf("%s/Segment_Masses.fig", subject_output_folderpath);
    saveas(fig_M, figname_M);
    figname_Izz = sprintf("%s/Segment_Inertias.fig", subject_output_folderpath);
    saveas(fig_Izz, figname_Izz);
    figname_CoM = sprintf("%s/Segment_Center_of_Mass_Positions.fig", subject_output_folderpath);
    saveas(fig_CoM, figname_CoM);
        
    % Get transformation between forceplate and model base
    base_R_fp = base_R_markers * markers_R_fp;
    base_r_base_fp = base_R_markers * (markers_r_markers_fp - markers_r_markers_base);
    
    % Get the transported wrenches from the forceplate to the base frame
    TransportedForceplate = ForceplateRotateTranslate(base_R_fp, base_r_base_fp, fp_Forceplate_filt);
    
    % Also in forceplate frame
    % Store the outputs
    Calibration.(subj).dynamicIdentification.markers_R_fp = markers_R_fp;
    Calibration.(subj).dynamicIdentification.markers_r_markers_fp = markers_r_markers_fp_dim;
    Calibration.(subj).dynamicIdentification.base_R_fp = base_R_fp;
    Calibration.(subj).dynamicIdentification.base_r_base_fp = base_r_base_fp;
    Calibration.(subj).dynamicIdentification.TransportedForceplate = TransportedForceplate;
    Calibration.(subj).dynamicIdentification.humanModel = identifiedHumanModel;
    Calibration.(subj).dynamicIdentification.residuals = sol_resnorms;
    Calibration.(subj).dynamicIdentification.wrenches_rmse = wrenches_rmse;
    % Store the human model
    Calibration.(subj).humanModel = identifiedHumanModel;
    
    % Close all these figures
    close all;
end

%% Savename
calibration_output_filepath = sprintf("%s/%s.mat", common_output_folderpath, output_filename);
save(calibration_output_filepath, "Calibration");