%STEP1 performs the inverse kinematics and dynamic identification on the
%calibration data.
close all;
clc;

%% Define constants
% Number of subjects
NO_SUBJECTS = 7;

% Define the rotation matrix from the Markers frame to the Human frame
base_R_markers = roty(180);

% Define the rotation matrix from the Forceplate frame to the Markers frame
markers_R_fp = [[1;0;0], [0;0;1], [0;-1;0]];

% Define the rotation matrix from the Forceplate to the Human base frame
base_R_fp = base_R_markers * markers_R_fp;

% Calibration data filepath
calibration_filepath = "../processed_data/Calibration/calibration.mat";
Calibration = importdata(calibration_filepath, "Calibration");

% Output folder path
common_output_folderpath = "../processed_data/Calibration/";


%% Perform the computations

% Prealocate an array of calibrated human models
CalibratedHumanModels = [];

% For each subject
for numSubj = 2 : NO_SUBJECTS
% for numSubj = 4
    
    % Subject name
    subj = sprintf("S%d", numSubj);
    
    % Output folderpath
    subject_output_folderpath = sprintf("%s%s/", common_output_folderpath, subj);
    if ~exist(subject_output_folderpath, 'dir')
        mkdir(subject_output_folderpath);
    end
    
    % Get a default human model
    % HumanModel6DOF constructor parameters
    R = eye(3);
    p = zeros(3, 1);    
    HEIGHT = Calibration.(subj).HEIGHT;
    WEIGHT = Calibration.(subj).WEIGHT;
    
    % Create a default HumanModel6DOF (uses anthropometric tables)
    currHumanModel = HumanModel6DOF(R,p,WEIGHT,HEIGHT);        
    
    % Get number of samples
    nbSamples = Calibration.(subj).NumberSamples;
%     nbSamples = 1000;
    
    % Make a time vector rate
    SamplingTime = Calibration.(subj).SamplingTime;
    SamplingFrequency = Calibration.(subj).SamplingFrequency;
    time_vec = 0 : SamplingTime : (nbSamples-1)*SamplingTime;
    
    % Define filtering parameters for raw data
    filt_fs = SamplingFrequency;
    filt_cutoff = 5;
    filt_order = 5;
    
    % Prepare the markers structure
    % TRanslation of markers from the OptiTrack frame to the HumanModel6DOF base frame
    pMarkers = -mean(Calibration.(subj).Markers.BODY.RANK);
    
    % Translate and rotate markers
    Markers = MarkersTranslateRotate(pMarkers, base_R_markers, Calibration.(subj).Markers);
    
    % Filter Marker data
    Markers = MarkersFilter(Markers,filt_fs,filt_cutoff,filt_order);
    
    % Get Marker reference lengths
    L_markers = MarkersGetSegmentLengths(MarkersIndex(Markers, 1:nbSamples));
    
    % Get segment length information from markers
    % Mean marker L
    meanL_markers = mean(L_markers);
    stdL_markers = std(L_markers);
    bandL_markers = [meanL_markers - 3*stdL_markers; meanL_markers + 3*stdL_markers;];
    patchBandL_markers = [meanL_markers - 3*stdL_markers; meanL_markers - 3*stdL_markers; meanL_markers + 3*stdL_markers; meanL_markers + 3*stdL_markers];
    minL_markers = min(L_markers, 1);
    maxL_markers = max(L_markers, 1);
    rangeL_markers = [minL_markers; maxL_markers];
    
    % Get segment length information from anthropometric tables
    minL_AT = 0.85 * currHumanModel.L;
    maxL_AT = 1.15 * currHumanModel.L;
    patchBandL_AT = [minL_AT; minL_AT; maxL_AT; maxL_AT];
    
    % Bounds on segment lengths
    minL = min(bandL_markers(1, :), minL_AT);
    maxL = max(bandL_markers(2, :), maxL_AT);
    midrangeL = (minL + maxL) / 2;
    
    % Compare lengths
    titlestring = {...        
    sprintf("Segment lenths of subject %s:", subj);
    "- Extracted directly from Markers $L_{\rm Markers}$, with 3 standard deviation band,";
    "- Extracted from Anthropometric Tables $L_{\rm AT}$, with 20\% deviation band,";
    "- Extracted by Kinematic Identification $L_{\rm IK}$.";
    };
    fig_L_time = figure('Position', [100, 50, 1280, 720]);
    sgtitle(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    hold on;
    plotopts = [];
    plotopts.title = @(n) {sprintf('$L_%d$', n), 'interpreter', 'latex', 'fontsize', 13};
    plotopts.xlabel = @(n) {'$t$ [s]', 'interpreter', 'latex', 'fontsize', 13};
    plotopts.ylabel = @(n) {sprintf('$L_%d$ [m]', n), 'interpreter', 'latex', 'fontsize', 13};
    plot_vector_quantities_opts_shape(time_vec, L_markers.', [], plotopts, [2, 3], 'b', 'DisplayName', '$L_{\rm Markers}$');
    plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(meanL_markers, [2, 1]).', [], [], [2, 3], 'b', 'LineWidth', 2, 'DisplayName', '$m_{L_{\rm Markers}}$');    patch_vector_quantities_opts_shape(time_vec([1, end, end, 1]), patchBandL_markers.', [], [], [.1, .1, .5], [2,3], 'FaceAlpha', 0.1, 'DisplayName', '$m_{L_{\rm Markers}} \pm 3 \sigma_{L_{\rm Markers}}$');
    plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(currHumanModel.L, [2, 1]).', [], [], [2, 3], 'r', 'LineWidth', 2, 'DisplayName', '$L_{\rm AT}$');
    patch_vector_quantities_opts_shape(time_vec([1, end, end, 1]), patchBandL_AT.', [], [], [.5, .1, .1], [2,3], 'FaceAlpha', 0.1, 'DisplayName', '$L_{\rm AT} \pm 20 \%$');
    legend('interpreter', 'latex');
    
    % Output structures
    L_out_2_1 = zeros(1, 6);
    L_out_2_2 = zeros(1, 6);
    q_out_1_1 = zeros(6, nbSamples);
    q_out_1_2 = zeros(6, nbSamples);
    q_out_2_1 = zeros(6, nbSamples);
    q_out_2_2 = zeros(6, nbSamples);
    resnorms_1_1 = zeros(6, nbSamples);
    resnorms_1_2 = zeros(6, nbSamples);
    resnorms_2_1 = zeros(6, nbSamples);
    resnorms_2_2 = zeros(6, nbSamples);
    
    
    % Create IK objects
    nbSimultaneousIKSamples = 1;
    % This IK object will identify angles, sample by sample
    ikModel1 = InverseKinematicsHumanModel6DOF(nbSimultaneousIKSamples);
    % This IK object will identify lengths, across all samples
    ikModel2 = InverseKinematicsHumanModel6DOF(nbSamples, 2);
    
    % Solver parameters
    sol_opt = struct();
    sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
    sol_opt.regularity_check = true;
    % Silent        
    sol_opt.ipopt.print_level = 0;
    sol_opt.print_time = 0;
    sol_opt.verbose = 0;
    sol_opt.ipopt.sb ='yes';
    % Set the angle identifying IK object here
    ikModel1.opti.solver('ipopt', sol_opt);
    % Set the options of the lenght identifying IK object here so it will
    % print
    ikModel2.opti.solver('ipopt', sol_opt);

    %%% PERFORM INITIAL JOINT ANGLE IK on all samples
    % Display progress
    fprintf("Samples %05d/%05d.", 0, nbSamples);
    % For all samples
    for sample = 1 : nbSamples
        % Display progress        
        fprintf(strcat(repmat('\b', 1, 12), "%05d/%05d."), sample, nbSamples);
        
        % Initialize joint angles
        if sample ~= 1
            % With previous sample
            q0 = q_out_1_1(:, sample-1);
        else
            % With default value (approximating human stance joint angles)
            q0 = [pi/3;pi/3;-pi/6;-pi/6;pi/2;-pi/4];
        end
        
        % Instantiate the IK model with the anthropometric table segment lengths
        ikModel1 = ikModel1.instantiateParameters(MarkersIndex(Markers,sample), q0, currHumanModel.L, currHumanModel.LowerJointLimits, currHumanModel.UpperJointLimits);
        % Solve the IK with those lengths
        ikSol_1_1 = ikModel1.opti.solve();
        
        % Instantiate the IK model with the mean segment lengths determined from markers
        ikModel1 = ikModel1.instantiateParameters(MarkersIndex(Markers,sample), q0, meanL_markers, currHumanModel.LowerJointLimits, currHumanModel.UpperJointLimits);
        % Solve the IK with those lengths
        ikSol_1_2 = ikModel1.opti.solve();
        
        % Store outputs of IK with the anthropometric table segment lengths
        q_out_1_1(:, sample) = ikSol_1_1.value(ikModel1.q);
        resnorms_1_1(:, sample) = ikModel1.computeResidualNorms(ikSol_1_1).';
        % Store outputs of IK with the mean segment lengths determined from markers
        q_out_1_2(:, sample) = ikSol_1_2.value(ikModel1.q);
        resnorms_1_2(:, sample) = ikModel1.computeResidualNorms(ikSol_1_2).'; 
    end
    fprintf("\n");
    
    % Print the obtained resnorms
    fprintf("sum(sum(resnorms_1_1.^2)) = %.4f\n", sum(sum(resnorms_1_1.^2)));
    fprintf("sum(sum(resnorms_1_2.^2)) = %.4f\n", sum(sum(resnorms_1_2.^2)));
    
    %%% PERFORM THE SEGMENT LENGTH - JOINT ANGLE IK ITERATIONS
    % Set the initial joint angle guesses for the segment length identification
    q_out_2_1 = q_out_1_1;
    q_out_2_2 = q_out_2_1;
    % Define the number of back-forth IK iterations
    nbIKiterations = 1;
    
    % Start length-identification
    for numIteration = 1 : nbIKiterations
        
        % Instantiate segment length identification with joint angles
        % identified with the anthropometric tables lengths
        ikModel2 = ikModel2.instantiateParameters(MarkersIndex(Markers, 1:nbSamples), q_out_2_1, midrangeL, minL, maxL);
        % Solve this segment length identification
        ikSol_2_1 = ikModel2.opti.solve();
        
        % Instantiate segment length identification with joint angles identified 
        % with the mean segment lengths determined from markers
        ikModel2 = ikModel2.instantiateParameters(MarkersIndex(Markers, 1:nbSamples), q_out_2_2, midrangeL, minL, maxL);
        % Solve this segment length identification
        ikSol_2_2 = ikModel2.opti.solve();
        
        % Store the segment lengths
        L_out_2_1 = ikSol_2_1.value(ikModel2.L);
        L_out_2_2 = ikSol_2_2.value(ikModel2.L);

        % ONCE-AGAIN PERFORM JOINT ANGLE IK on all samples
        % Display progress
        fprintf("Samples %05d/%05d.", 0, nbSamples);
        for sample = 1 : nbSamples
            % Display progress        
            fprintf(strcat(repmat('\b', 1, 12), "%05d/%05d."), sample, nbSamples);

            % Initialize joint angles according to previous iteration results
            q0_1 = q_out_1_1(:, sample);
            q0_2 = q_out_1_2(:, sample);

            % Instantiate IK model with the identified segment lengths 2_1
            ikModel1 = ikModel1.instantiateParameters(MarkersIndex(Markers,sample), q0_1, L_out_2_1, currHumanModel.LowerJointLimits, currHumanModel.UpperJointLimits);
            % Solve the IK with those lengths
            ikSol1_2_1 = ikModel1.opti.solve();
            
            % Instantiate with different length parameter
            ikModel1 = ikModel1.instantiateParameters(MarkersIndex(Markers,sample), q0_2, L_out_2_2, currHumanModel.LowerJointLimits, currHumanModel.UpperJointLimits);
            % Solve the IK with those lengths
            ikSol1_2_2 = ikModel1.opti.solve();

            % Store outputs
            q_out_2_1(:, sample) = ikSol1_2_1.value(ikModel1.q);
            resnorms_2_1(:, sample) = ikModel1.computeResidualNorms(ikSol1_2_1).';
            % Store outputs
            q_out_2_2(:, sample) = ikSol1_2_2.value(ikModel1.q);
            resnorms_2_2(:, sample) = ikModel1.computeResidualNorms(ikSol1_2_2).'; 
        end
        fprintf("\n");
        
        fprintf("sum(sum(resnorms_2_1.^2)) = %.4f\n", sum(sum(resnorms_2_1.^2)));
        fprintf("sum(sum(resnorms_2_2.^2)) = %.4f\n", sum(sum(resnorms_2_2.^2)));
    end
    
    % Animate 2_1
    fig_anim = figure('Position', [100, 50, 1280, 720]);
    animopts = [];
    animopts.title = sprintf("Animation of kinematically identified model alongside Markers");
%     animopts.show_legend = true;
    animopts.save_path = sprintf("%sKinematic_Identification", subject_output_folderpath);
    Animate_nDOF_Markers(q_out_2_1, MarkersIndex(Markers, 1:nbSamples), L_out_2_1, 0.01, animopts);
    
    % Compare resnorms
    titlestring = {...        
    sprintf("Distances between Markers and predicted joint frame locations of subject %s:", subj);
    "- with segment lengths extracted directly from Markers ${\rm IK}_{\rm Markers}$,";
    "- with segment lengths extracted from Anthropometric Tables ${\rm IK}_{\rm AT}$";
    "- with segment lengths extracted by Kinematic Identification initialized with Markers ${\rm IK}_{OPT-1}$.";
    "- with segment lengths extracted by Kinematic Identification initialized with Anthropometric Tables ${\rm IK}_{OPT-2}$.";
    };
    fig_kin_residuals = figure('Position', [100, 50, 1280, 720]);
    sgtitle(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    hold on;
    names = ["Knee", "Hip (GTR)", "Back", "Shoulder", "Elbow", "Wrist"];
    plotopts = [];
    plotopts.title = @(n) {sprintf("%s Marker", names(n)), 'interpreter', 'latex', 'fontsize', 13};
    plotopts.xlabel = @(n) {"$t$ [s]", 'interpreter', 'latex', 'fontsize', 13};
    plotopts.ylabel = @(n) {sprintf("${\\rm RES}_{%s}$ [m]", names(n)), 'interpreter', 'latex', 'fontsize', 13};
    plot_vector_quantities_opts_shape(time_vec, resnorms_1_1, [], plotopts, [2, 3], 'DisplayName', "${\rm IK}_{\rm Markers}$");
    plot_vector_quantities_opts_shape(time_vec, resnorms_1_2, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm AT}$");
    plot_vector_quantities_opts_shape(time_vec, resnorms_2_1, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm OPT-1}$");
    plot_vector_quantities_opts_shape(time_vec, resnorms_2_2, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm OPT-2}$");
    legend('interpreter', 'latex');
    fprintf("sum(sum(resnorms_1_1.^2)) = %.4f\n", sum(sum(resnorms_1_1.^2)));
    fprintf("sum(sum(resnorms_1_2.^2)) = %.4f\n", sum(sum(resnorms_1_2.^2)));
    fprintf("sum(sum(resnorms_2_1.^2)) = %.4f\n", sum(sum(resnorms_2_1.^2)));
    fprintf("sum(sum(resnorms_2_2.^2)) = %.4f\n", sum(sum(resnorms_2_2.^2)));
    
    % Update
    if sum(sum(resnorms_2_1.^2)) < sum(sum(resnorms_2_2.^2))
        calibratedHumanModel.L = L_out_2_1;
        calibratedHumanModel.q = q_out_2_1;
        % Segment lengths
        figure(fig_L_time);
        plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(L_out_2_1.', [1, 2]), [], [], [2, 3], 'g', 'LineWidth', 2, 'DisplayName', '$L_{IK}$');
    else
        calibratedHumanModel.L = L_out_2_2;
        calibratedHumanModel.q = q_out_2_2;
        % Segment lengths
        figure(fig_L_time);
        plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(L_out_2_2.', [1, 2]), [], [], [2, 3], 'g', 'LineWidth', 2, 'DisplayName', '$L_{IK}$');        
    end
    CalibratedHumanModels = [CalibratedHumanModels; calibratedHumanModel];
    
    % Plot the lengths
    % Names of the body parts
    names = ["Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms"];
    % Data vector
    lengths = [...
    [meanL_markers];
    [currHumanModel.L];
    [calibratedHumanModel.L];
    ].';
    % Figure plot
    fig_L = figure('Position', [100, 50, 1280, 720]);
    lengthbar = bar(lengths);
    % Aesthetics
    xticklabels(names);
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
    xlabel("Body parts", 'interpreter', 'latex', 'fontsize', 13);
    ylabel("Segment lengths [m]", 'interpreter', 'latex', 'fontsize', 13);
    legend("Directly from Markers", "Anthropometric Tables", "Kinematic Identification", 'interpreter', 'latex', 'fontsize', 13);
    % Title
    titlestring = {sprintf("Subject %s segment lengths:", subj)};
    title(titlestring, 'interpreter', 'latex', 'fontsize', 13);
    
    % Update model lenghts
    currHumanModel.L = calibratedHumanModel.L;
    
    %% Dynamic identification
    % Get velocities and accelerations
    % Filter q, and find dq and ddq
    q_filt = lowpass_filter(calibratedHumanModel.q, filt_fs, filt_cutoff, filt_order);
    dq_filt = diff(q_filt, 1, 2) ./ SamplingTime;
    ddq_filt = diff(q_filt, 2, 2) ./ SamplingTime^2;
    dq_filt = [dq_filt, dq_filt(:, end)];
    ddq_filt = [ddq_filt, ddq_filt(:, end-1:end)];
    
    % Lowpass filter the forceplate data
    fp_Forceplate_filt = ForceplateFilter(Calibration.(subj).Forceplate, filt_fs, filt_cutoff, filt_order);
    % Index the forceplate data
    fp_Forceplate_filt = ForceplateIndex(fp_Forceplate_filt, 1:nbSamples);
    % Define the vector from markers to base frame in markers frame
    markers_r_markers_base = mean(Calibration.(subj).Markers.BODY.RANK).';
    % Define the supposed vector from markers to forceplate frame in
    % markers frame ( Center of Forceplate )
    markers_r_markers_fp = [0.2; 0; 0.3];
    % Calculate the vector from base frame to forceplate frame in markers frame
    markers_r_base_fp = markers_r_markers_base - markers_r_markers_fp;
    % Calculate the vector from base frame to forceplate frame in base frame
    base_r_base_fp = base_R_markers * markers_r_base_fp;
    % Calculate the transposed forceplate data
    base_Forceplate_filt = ForceplateRotateTranslate(base_R_fp, base_r_base_fp, fp_Forceplate_filt);
    % Extract in grf and cop form
    f_grf_fp = [base_Forceplate_filt.Forces.'; base_Forceplate_filt.Moments.'];
    cop_fp = base_Forceplate_filt.COP(:, 1).';
    
    % Define external forces (there are none)
    fFOOT = zeros(6, nbSamples);
    fHAND = zeros(6, nbSamples);
    % Calculate the ground reaction forces
    [tau, f_grf_model] = currHumanModel.inverseDynamicModel(q_filt, dq_filt, ddq_filt, fFOOT, fHAND);
    % Calculate the COP position
    cop_model = f_grf_model(6, :) ./ f_grf_model(2, :);
    
    %% Do the dynamic identification    
    dimModel = DynamicIdentificationHumanModel6DOF(nbSamples);
    % Initialize the search
    dimModel = dimModel.instantiateParameters(fp_Forceplate_filt, q_filt, dq_filt, ddq_filt, currHumanModel, base_R_fp, base_r_base_fp);
    % Solver choice
    dimModel.opti.solver('ipopt');
    % Solution
    dim_sol = dimModel.opti.solve();
    % Get the ground reaction forces and cop
    f_grf_dim = dimModel.computeGRF(dim_sol);    
    cop_dim = f_grf_dim(6, :) ./ f_grf_dim(2, :);
    
    %% Display and Plot the residual of the GRFs and COPs and
    % Residuals of the anthropometric tables model
    res_fx = f_grf_model(1, :).' - f_grf_fp(1, :).';
    res_fy = f_grf_model(2, :).' - f_grf_fp(2, :).';
    res_mz = f_grf_model(6, :).' - f_grf_fp(6, :).';
    res_cop = cop_model.' - cop_fp.';
    fprintf("sqrt(sum(sum(res_fx.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(res_fx.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(res_fy.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(res_fy.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(res_mz.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(res_mz.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(res_cop.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(res_cop.^2) ./ (2 * nbSamples))));
    % Residual of the dynamically identified model
    sol_resnorms = dimModel.computeResidualNorms(dim_sol);
    sol_res_fx = sol_resnorms(:, 1);
    sol_res_fy = sol_resnorms(:, 2);
    sol_res_mz = sol_resnorms(:, 3);
    sol_res_cop = sol_resnorms(:, 4);
    fprintf("sqrt(sum(sum(sol_res_fx.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(sol_res_fx.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(sol_res_fy.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(sol_res_fy.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(sol_res_mz.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(sol_res_mz.^2) ./ (2 * nbSamples))));
    fprintf("sqrt(sum(sum(sol_res_cop.^2) ./ (2 * nbSamples))) = %.4f\n", ...
             sqrt(sum(sum(sol_res_cop.^2) ./ (2 * nbSamples))));

    % Stack the residuals in a list
    titlestring = {...
    sprintf("Residuals of force, torque and CoM predictions of the model of subject %s with respect to the forceplate measurements, with and without dynamic identification.", subj);
    strcat( ...
    sprintf("$R_{M_x} = \\sqrt{\\frac{1}{T} \\sum ( (f_{\\rm DIM Model})_x - (f_{\\rm Forceplate})_x)^2} = %.4f$ [N]", ...
             sqrt(sum(sum(sol_res_fx.^2) ./ nbSamples))), ...
    sprintf("$R_{M_y} = \\sqrt{\\frac{1}{T} \\sum ( (f_{\\rm DIM Model})_y - (f_{\\rm Forceplate})_y)^2} = %.4f$ [N]\n", ...
             sqrt(sum(sum(sol_res_fy.^2) ./ nbSamples))), ...             
    sprintf("$R_{M_z} = \\sqrt{\\frac{1}{T} \\sum ( (\\tau_{\\rm DIM Model})_z - (\\tau_{\\rm Forceplate})_z)^2} = %.4f$ [N.m]", ...
             sqrt(sum(sum(sol_res_mz.^2) ./ nbSamples))), ...
    sprintf("$R_{{\\rm COP}_x} = \\sqrt{\\frac{1}{T} \\sum ( ({\\rm COP}_{\\rm DIM Model})_x - ({\\rm COP}_{\\rm Forceplate})_x)^2} = %.4f$ [m]", ...
             sqrt(sum(sum(sol_res_cop.^2) ./ nbSamples))) ...
    );
    };
    % Plot the GRF and COP
    fig_dyn_residuals = figure('Position', [100, 50, 1280, 720]);
    sgtitle(titlestring, 'fontsize', 13, 'interpreter', 'latex');
    names = ["$R_{F_x}$", "$R_{F_y}$", "$R_{M_z}$", '$R_{{\rm COP}_x}$'];
    units = ["N", "N", "N.m", "m"];
    plotopts = [];
    plotopts.title = @(n) {names(n), 'interpreter', 'latex', 'fontsize', 13};
    plotopts.xlim = @(n){[time_vec(1), time_vec(end)]};
    plotopts.xlabel = @(n) {"$t$[s]", 'interpreter', 'latex', 'fontsize', 13};
    plotopts.ylabel = @(n) {sprintf("%s [%s]", names(n), units(n)), 'interpreter', 'latex', 'fontsize', 13};
    plot_vector_quantities_opts_shape(time_vec, [f_grf_model([1,2,6], :); cop_model], [], plotopts, [2, 2], 'b', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm AT Model}$');
    plot_vector_quantities_opts_shape(time_vec, [f_grf_fp([1,2,6], :); cop_fp], [], [], [2, 2], 'r', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm Forceplate}$');
    plot_vector_quantities_opts_shape(time_vec, [f_grf_dim([1,2,6], :); cop_dim], [], [], [2, 2], 'g', 'DisplayName', '$(f, \tau, {\rm COP})_{\rm DIM Model}$');
    legend('interpreter', 'latex', 'location', 'best');
    
    %% Load the numeric model result
    % Load the human model parameters
    identifiedHumanModel = dimModel.computeNumericalModel(dim_sol);
    % Load the 
    identified_base_r_base_fp = dimModel.compute_base_r_base_fp(dim_sol);
    identified_markers_r_base_fp = base_R_markers.' * identified_base_r_base_fp;
    identified_markers_r_markers_fp = markers_r_markers_base + identified_markers_r_base_fp;
    markers_r_markers_fp
    identified_markers_r_markers_fp
    %% Barplot the retrieved parameters
    
    % Plot the masses
    % Names of the body parts
    names = ["Feet", "Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms", "Hands", "Neck-Head"];
    % Data vector
    mass = [...
    [currHumanModel.MFOOT, currHumanModel.M, currHumanModel.MHAND, currHumanModel.MHEAD];
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
    [currHumanModel.IzzFOOT, currHumanModel.Izz, currHumanModel.IzzHAND, currHumanModel.IzzHEAD];
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
    [currHumanModel.CoMFOOT(1), currHumanModel.CoM(1, :), currHumanModel.CoMHAND(1), currHumanModel.CoMHEAD(1)];
    [identifiedHumanModel.CoMFOOT(1), identifiedHumanModel.CoM(1, :), identifiedHumanModel.CoMHAND(1), identifiedHumanModel.CoMHEAD(1)];
    ].';
    comsy = [...
    [currHumanModel.CoMFOOT(2), currHumanModel.CoM(2, :), currHumanModel.CoMHAND(2), currHumanModel.CoMHEAD(2)];
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
    figname_L_time = sprintf("%sSegment_Lenths_in_Time.fig", subject_output_folderpath);
    saveas(fig_L_time, figname_L_time);
    figname_L = sprintf("%sSegment_Lenths.fig", subject_output_folderpath);
    saveas(fig_L, figname_L);
    figname_kin_residuals = sprintf("%sKinematic_Residuals.fig", subject_output_folderpath);
    saveas(fig_kin_residuals, figname_kin_residuals);
    figname_dyn_residuals = sprintf("%sDynamic_Residuals.fig", subject_output_folderpath);
    saveas(fig_dyn_residuals, figname_dyn_residuals);
    figname_M = sprintf("%sSegment_Masses.fig", subject_output_folderpath);
    saveas(fig_M, figname_M);
    figname_Izz = sprintf("%sSegment_Inertias.fig", subject_output_folderpath);
    saveas(fig_Izz, figname_Izz);
    figname_CoM = sprintf("%sSegment_Center_of_Mass_Positions.fig", subject_output_folderpath);
    saveas(fig_CoM, figname_CoM);
    
    % Store results
    Calibration.(subj).humanModel = identifiedHumanModel;
    Calibration.(subj).q = q_filt;
end

% Savename
calibration_output_filepath = sprintf("%scalibrated.mat", common_output_folderpath);
save(calibration_output_filepath, "Calibration");
%%

% %%
% fprintf("base_r_base_fp = [%.4f, %.4f, %.4f]\n", ...
%          base_r_base_fp);
% fprintf("dimModel.opti.debug.value(dimModel.base_r_base_fp, dimModel.opti.initial()) = [%.4f, %.4f, %.4f]\n", ...
%         dimModel.opti.debug.value(dimModel.base_r_base_fp, dimModel.opti.initial()) ...
%         );
%     
% fprintf("base_R_fp =  [%.4f, %.4f, %.4f] [%.4f, %.4f, %.4f] [%.4f, %.4f, %.4f]\n", ...
%          base_R_fp);
% fprintf("dimModel.opti.debug.value(dimModel.base_R_fp, dimModel.opti.initial()) = [%.4f, %.4f, %.4f] [%.4f, %.4f, %.4f] [%.4f, %.4f, %.4f]\n", ...
%         dimModel.opti.debug.value(dimModel.base_R_fp, dimModel.opti.initial()) ...
%         );
%     
% %%
% fprintf("all(base_Forceplate_filt.Forces - dimModel.opti.debug.value(dimModel.TransportedForceplate.Forces, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(base_Forceplate_filt.Forces - dimModel.opti.debug.value(dimModel.TransportedForceplate.Forces, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(base_Forceplate_filt.Moments - dimModel.opti.debug.value(dimModel.TransportedForceplate.Moments, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(base_Forceplate_filt.Moments - dimModel.opti.debug.value(dimModel.TransportedForceplate.Moments, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(base_Forceplate_filt.COP - dimModel.opti.debug.value(dimModel.TransportedForceplate.COP, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(base_Forceplate_filt.COP - dimModel.opti.debug.value(dimModel.TransportedForceplate.COP, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% 
% %% Check model
% fprintf("all(currHumanModel.M - dimModel.opti.debug.value(dimModel.M, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.M - dimModel.opti.debug.value(dimModel.M, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.CoM - dimModel.opti.debug.value(dimModel.CoM, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.CoM - dimModel.opti.debug.value(dimModel.CoM, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.Izz - dimModel.opti.debug.value(dimModel.Izz, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.Izz - dimModel.opti.debug.value(dimModel.Izz, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.MFOOT - dimModel.opti.debug.value(dimModel.MFOOT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.MFOOT - dimModel.opti.debug.value(dimModel.MFOOT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.MHAND - dimModel.opti.debug.value(dimModel.MHAND, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.MHAND - dimModel.opti.debug.value(dimModel.MHAND, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.MHEAD - dimModel.opti.debug.value(dimModel.MHEAD, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.MHEAD - dimModel.opti.debug.value(dimModel.MHEAD, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.IzzFOOT - dimModel.opti.debug.value(dimModel.IzzFOOT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.IzzFOOT - dimModel.opti.debug.value(dimModel.IzzFOOT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.IzzHAND - dimModel.opti.debug.value(dimModel.IzzHAND, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.IzzHAND - dimModel.opti.debug.value(dimModel.IzzHAND, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.IzzHEAD - dimModel.opti.debug.value(dimModel.IzzHEAD, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.IzzHEAD - dimModel.opti.debug.value(dimModel.IzzHEAD, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.CoMFOOT - dimModel.opti.debug.value(dimModel.CoMFOOT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.CoMFOOT - dimModel.opti.debug.value(dimModel.CoMFOOT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.CoMHAND - dimModel.opti.debug.value(dimModel.CoMHAND, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.CoMHAND - dimModel.opti.debug.value(dimModel.CoMHAND, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.CoMHEAD - dimModel.opti.debug.value(dimModel.CoMHEAD, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.CoMHEAD - dimModel.opti.debug.value(dimModel.CoMHEAD, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% 
% 
% fprintf("all(currHumanModel.WEIGHT - dimModel.opti.debug.value(dimModel.casadiHumanModel.WEIGHT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.WEIGHT - dimModel.opti.debug.value(dimModel.casadiHumanModel.WEIGHT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.HEIGHT - dimModel.opti.debug.value(dimModel.casadiHumanModel.HEIGHT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.HEIGHT - dimModel.opti.debug.value(dimModel.casadiHumanModel.HEIGHT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.R - dimModel.opti.debug.value(dimModel.casadiHumanModel.R, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.R - dimModel.opti.debug.value(dimModel.casadiHumanModel.R, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.p - dimModel.opti.debug.value(dimModel.casadiHumanModel.p, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.p - dimModel.opti.debug.value(dimModel.casadiHumanModel.p, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.L - dimModel.opti.debug.value(dimModel.casadiHumanModel.L, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.L - dimModel.opti.debug.value(dimModel.casadiHumanModel.L, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.HeelPosition - dimModel.opti.debug.value(dimModel.casadiHumanModel.HeelPosition, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.HeelPosition - dimModel.opti.debug.value(dimModel.casadiHumanModel.HeelPosition, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.ToePosition - dimModel.opti.debug.value(dimModel.casadiHumanModel.ToePosition, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.ToePosition - dimModel.opti.debug.value(dimModel.casadiHumanModel.ToePosition, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.LowerJointLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.LowerJointLimits, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.LowerJointLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.LowerJointLimits, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.UpperJointLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.UpperJointLimits, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.UpperJointLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.UpperJointLimits, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.LowerTorqueLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.LowerTorqueLimits, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.LowerTorqueLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.LowerTorqueLimits, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.UpperTorqueLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.UpperTorqueLimits, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.UpperTorqueLimits - dimModel.opti.debug.value(dimModel.casadiHumanModel.UpperTorqueLimits, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(currHumanModel.LengthToRadiiFactor - dimModel.opti.debug.value(dimModel.casadiHumanModel.LengthToRadiiFactor, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(currHumanModel.LengthToRadiiFactor - dimModel.opti.debug.value(dimModel.casadiHumanModel.LengthToRadiiFactor, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% %% Check angles and external forces
% 
% fprintf("all(q_filt - dimModel.opti.debug.value(dimModel.q, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(q_filt - dimModel.opti.debug.value(dimModel.q, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(dq_filt - dimModel.opti.debug.value(dimModel.dq, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(dq_filt - dimModel.opti.debug.value(dimModel.dq, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(ddq_filt - dimModel.opti.debug.value(dimModel.ddq, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(ddq_filt - dimModel.opti.debug.value(dimModel.ddq, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(fFOOT - dimModel.opti.debug.value(dimModel.fFOOT, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(fFOOT - dimModel.opti.debug.value(dimModel.fFOOT, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% fprintf("all(fHAND - dimModel.opti.debug.value(dimModel.fHAND, dimModel.opti.initial()) < 1e-12, 'all') = %d\n", ...
%     all(fHAND - dimModel.opti.debug.value(dimModel.fHAND, dimModel.opti.initial()) < 1e-12, 'all') ...
%     );
% %%
% fprintf("all(f_grf_model - dimModel.opti.debug.value(dimModel.f_grf, dimModel.opti.initial()) < 1e-2, 'all') = %d\n", ...
%     all(f_grf_model - dimModel.opti.debug.value(dimModel.f_grf, dimModel.opti.initial()) < 1e-2, 'all') ...
%     );
% 
% %%
% res_fx = f_grf_model(1, :).' - f_grf_fp(1, :).';
% res_fy = f_grf_model(2, :).' - f_grf_fp(2, :).';
% res_mz = f_grf_model(6, :).' - f_grf_fp(6, :).';
% fprintf("sum(sum(res_fx.^2) ./ (2 * nbSamples)) = %.4f\n", ...
%          sum(sum(res_fx.^2) ./ (2 * nbSamples)));
% fprintf("sum(sum(res_fy.^2) ./ (2 * nbSamples)) = %.4f\n", ...
%          sum(sum(res_fy.^2) ./ (2 * nbSamples)));
% fprintf("sum(sum(res_mz.^2) ./ (2 * nbSamples)) = %.4f\n", ...
%          sum(sum(res_mz.^2) ./ (2 * nbSamples)));
%      
% 
% fprintf("dimModel.opti.debug.value(dimModel.cf_fx, dimModel.opti.initial()) = %.4f\n", ...
%          dimModel.opti.debug.value(dimModel.cf_fx, dimModel.opti.initial()));
% fprintf("dimModel.opti.debug.value(dimModel.cf_fy, dimModel.opti.initial()) = %.4f\n", ...
%          dimModel.opti.debug.value(dimModel.cf_fy, dimModel.opti.initial()));
% fprintf("dimModel.opti.debug.value(dimModel.cf_mz, dimModel.opti.initial()) = %.4f\n", ...
%          dimModel.opti.debug.value(dimModel.cf_mz, dimModel.opti.initial()));
