%STEP1 performs the inverse kinematics and dynamic identification on the
%calibration data.
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
calibration_filepath = "../processed_data/Calibration/calibration.mat";
Calibration = importdata(calibration_filepath, "Calibration");

% Output folder path
common_output_folderpath = "../processed_data/Calibration";

% Output filename
output_filename = "kinematically_calibrated";


%% Perform the computations

% For each subject
for numSubj = 2 : NO_SUBJECTS
    
    % Subject name
    subj = sprintf("S%d", numSubj);
    
    % Output folderpath
    subject_output_folderpath = sprintf("%s/%s", common_output_folderpath, subj);
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
    defaultHumanModel = HumanModel6DOF(R,p,WEIGHT,HEIGHT);        
    
    % Get number of samples
    nbSamples = Calibration.(subj).NumberSamples;
    
    % Make a time vector rate
    SamplingTime = Calibration.(subj).SamplingTime;
    SamplingFrequency = Calibration.(subj).SamplingFrequency;
    time_vec = 0 : SamplingTime : (nbSamples-1)*SamplingTime;
    
    % Define filtering parameters for raw data
    filt_fs = SamplingFrequency;
    filt_cutoff = 5;
    filt_order = 5;
    
    % Prepare the markers structure
    % Filter the markers
    Markers = MarkersFilter(Calibration.(subj).Markers, filt_fs, filt_cutoff, filt_order);
    % Extract position vector of the HumanModel6DOF base frame in the OptiTrack frame
    markers_r_markers_base = mean(Markers.BODY.RANK);
    % Translate and rotate markers
    TransportedMarkers = MarkersTranslateRotate(-markers_r_markers_base, base_R_markers, Markers);
    
    % Extract heel and toe position
    HeelPosition = mean(Markers.BODY.RHEE - Markers.BODY.RANK).';
    ToePosition = mean(Markers.BODY.RTOE - Markers.BODY.RANK).';
    defaultHumanModel.HeelPosition = HeelPosition(1:2);
    defaultHumanModel.ToePosition = ToePosition(1:2);
    
    % Get segment length information from markers
    % Get reference segment lengths as the lengths between successive markers
    L_markers = MarkersGetSegmentLengths(MarkersIndex(TransportedMarkers, 1:nbSamples));
    % Get reference segment lenth statistics
    meanL_markers = mean(L_markers);
    stdL_markers = std(L_markers);
    bandL_markers = [meanL_markers - 3*stdL_markers; meanL_markers + 3*stdL_markers;];
    patchBandL_markers = [meanL_markers - 3*stdL_markers; meanL_markers - 3*stdL_markers; meanL_markers + 3*stdL_markers; meanL_markers + 3*stdL_markers];
    minL_markers = min(L_markers, 1);
    maxL_markers = max(L_markers, 1);
    rangeL_markers = [minL_markers; maxL_markers];
    
    % Get segment length information from anthropometric tables
    minL_AT = 0.85 * defaultHumanModel.L;
    maxL_AT = 1.15 * defaultHumanModel.L;
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
    plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(defaultHumanModel.L, [2, 1]).', [], [], [2, 3], 'r', 'LineWidth', 2, 'DisplayName', '$L_{\rm AT}$');
    patch_vector_quantities_opts_shape(time_vec([1, end, end, 1]), patchBandL_AT.', [], [], [.5, .1, .1], [2,3], 'FaceAlpha', 0.1, 'DisplayName', '$L_{\rm AT} \pm 20 \%$');
    legend('interpreter', 'latex');
    
    
    % Prepare kinematic identification
    
    % Output structures
    L_out_2_1 = zeros(1, 6);
    L_out_2_2 = zeros(1, 6);
    q_out_1_1 = zeros(6, nbSamples);
    q_out_1_2 = zeros(6, nbSamples);
    q_out_2_1 = zeros(6, nbSamples);
    q_out_2_2 = zeros(6, nbSamples);
    residuals_1_1 = zeros(6, nbSamples);
    residuals_1_2 = zeros(6, nbSamples);
    residuals_2_1 = zeros(6, nbSamples);
    residuals_2_2 = zeros(6, nbSamples);    
    % Create IK objects
    % This IK object will identify angles, sample by sample
    ikModeFlag = 1;
    nbSimultaneousIKSamples = 1;
    ikModel1 = InverseKinematicsHumanModel6DOF(nbSimultaneousIKSamples, ikModeFlag);
    % This IK object will identify lengths, across all samples
    ikModeFlag = 2;
    ikModel2 = InverseKinematicsHumanModel6DOF(nbSamples, ikModeFlag);
    
    % Solver parameters
    sol_opt = struct();
    % Regularity checks
    sol_opt.ipopt.check_derivatives_for_naninf = 'yes';
    sol_opt.regularity_check = true;
    % Silent solver (no outputs)
    sol_opt.ipopt.print_level = 0;
    sol_opt.print_time = 0;
    sol_opt.verbose = 0;
    sol_opt.ipopt.sb ='yes';
    % Set the options for the angle-identifying IK object here
    ikModel1.opti.solver('ipopt', sol_opt);
    % Set the options of the length identifying IK object here
    ikModel2.opti.solver('ipopt', sol_opt);

    %%% PERFORM INITIAL JOINT ANGLE IK on all samples
    fprintf("IK iteration 0:\t");
    % Display progress
    fprintf("Samples %05d/%05d.", 0, nbSamples);
    % For all samples
    for sample = 1 : nbSamples
        % Display progress        
        fprintf(strcat(repmat('\b', 1, 12), "%05d/%05d."), sample, nbSamples);
        
        % Initialize joint angles
        if sample ~= 1
            % If it is not the first sample, initialize with previous sample
            q0 = q_out_1_1(:, sample-1);
            % And normalize the joint velocities
            ikVelocityNormalizationConstant = 1;
        else
            % If it is the first sample, initialize with default value (approximating human stance joint angles)
            q0 = [pi/3;pi/3;-pi/6;-pi/6;pi/2;-pi/4];
            % do not normalize the velocities
            ikVelocityNormalizationConstant = 0;
        end
        
        % Instantiate the IK model with the anthropometric table segment lengths
        ikModel1 = ikModel1.instantiateParameters(MarkersIndex(TransportedMarkers,sample), q0, defaultHumanModel.L, ikVelocityNormalizationConstant, defaultHumanModel.LowerJointLimits, defaultHumanModel.UpperJointLimits);
        % Solve the IK with those lengths
        ikSol_1_1 = ikModel1.opti.solve();
        
        % Instantiate the IK model with the mean segment lengths determined from markers
        ikModel1 = ikModel1.instantiateParameters(MarkersIndex(TransportedMarkers,sample), q0, meanL_markers, ikVelocityNormalizationConstant, defaultHumanModel.LowerJointLimits, defaultHumanModel.UpperJointLimits);
        % Solve the IK with those lengths
        ikSol_1_2 = ikModel1.opti.solve();
        
        % Store outputs of IK with the anthropometric table segment lengths
        q_out_1_1(:, sample) = ikSol_1_1.value(ikModel1.q);
        residuals_1_1(:, sample) = ikModel1.computeResidualNorms(ikSol_1_1).';
        % Store outputs of IK with the mean segment lengths determined from markers
        q_out_1_2(:, sample) = ikSol_1_2.value(ikModel1.q);
        residuals_1_2(:, sample) = ikModel1.computeResidualNorms(ikSol_1_2).'; 
    end
    fprintf("\n");
    
    % Print the obtained residuals
    fprintf("sum(sqrt(sum(residuals_1_1.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_1_1.^2, 2) ./ nbSamples)));
    fprintf("sum(sqrt(sum(residuals_1_2.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_1_2.^2, 2) ./ nbSamples)));
    
    %%% PERFORM THE SEGMENT LENGTH - JOINT ANGLE IK ITERATIONS
    % Set the initial joint angle guesses for the segment length identification
    q_out_2_1 = q_out_1_1;
    q_out_2_2 = q_out_1_2;
    % Initialize mean square errors
    mse_2_1 = sum(sum(residuals_1_1.^2) ./ nbSamples);
    mse_2_2 = sum(sum(residuals_1_2.^2) ./ nbSamples);
    % Fix lower factor decrease threshold
    lowerDecreaseFactorThreshold = 0.99;
    % Define the number of back-forth IK iterations
    nbMaxIKiterations = 5;
    
    % Start length-identification
    for numIteration = 1 : nbMaxIKiterations
        
        % Display IK iteration
        fprintf("IK Iteration %d:\t", numIteration);
        
        % For the length optimization normalize the velocities
        ikVelocityNormalizationConstant = 1;
        % Instantiate segment length identification with joint angles
        % identified with the anthropometric tables lengths
        ikModel2 = ikModel2.instantiateParameters(MarkersIndex(TransportedMarkers, 1:nbSamples), q_out_2_1, midrangeL, ikVelocityNormalizationConstant, minL, maxL);
        % Solve this segment length identification
        ikSol_2_1 = ikModel2.opti.solve();
        
        % Instantiate segment length identification with joint angles identified 
        % with the mean segment lengths determined from markers
        ikModel2 = ikModel2.instantiateParameters(MarkersIndex(TransportedMarkers, 1:nbSamples), q_out_2_2, midrangeL, ikVelocityNormalizationConstant, minL, maxL);
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
            if sample ~= 1
                % When the sample is not the first, initialize with previous
                q0_1 = q_out_2_1(:, sample-1);
                q0_2 = q_out_2_2(:, sample-1);
                % Normalize velocities
                ikVelocityNormalizationConstant = 1;
            else
                % When the sample is the first, initialize with previous sample
                q0_1 = q_out_2_1(:, sample);
                q0_2 = q_out_2_2(:, sample);
                % Do not normalize the velocities
                ikVelocityNormalizationConstant = 0;
            end

            % Instantiate IK model with the identified segment lengths 2_1
            ikModel1 = ikModel1.instantiateParameters(MarkersIndex(TransportedMarkers,sample), q0_1, L_out_2_1, ikVelocityNormalizationConstant, defaultHumanModel.LowerJointLimits, defaultHumanModel.UpperJointLimits);
            % Solve the IK with those lengths
            ikSol1_2_1 = ikModel1.opti.solve();
            
            % Instantiate with different length parameter
            ikModel1 = ikModel1.instantiateParameters(MarkersIndex(TransportedMarkers,sample), q0_2, L_out_2_2, ikVelocityNormalizationConstant, defaultHumanModel.LowerJointLimits, defaultHumanModel.UpperJointLimits);
            % Solve the IK with those lengths
            ikSol1_2_2 = ikModel1.opti.solve();

            % Store outputs
            q_out_2_1(:, sample) = ikSol1_2_1.value(ikModel1.q);
            residuals_2_1(:, sample) = ikModel1.computeResidualNorms(ikSol1_2_1).';
            % Store outputs
            q_out_2_2(:, sample) = ikSol1_2_2.value(ikModel1.q);
            residuals_2_2(:, sample) = ikModel1.computeResidualNorms(ikSol1_2_2).'; 
        end
        fprintf("\n");
        
        % Calculate MSE
        new_mse_2_1 = sum(sum(residuals_2_1.^2) ./ nbSamples);
        new_mse_2_2 = sum(sum(residuals_2_2.^2) ./ nbSamples);
        % If mse decreased by less than 1 percent break
        decreaseQuotientVector = [(new_mse_2_1 / mse_2_1), (new_mse_2_2 / mse_2_2)];
        if min(decreaseQuotientVector) > lowerDecreaseFactorThreshold
            break;
        else
            fprintf("Decrease factor: %.2f \n", 1-min(decreaseQuotientVector));
            mse_2_1 = new_mse_2_1;
            mse_2_2 = new_mse_2_2;
        end
        fprintf("sum(sqrt(sum(residuals_2_1.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_2_1.^2, 2) ./ nbSamples)));
        fprintf("sum(sqrt(sum(residuals_2_2.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_2_2.^2, 2) ./ nbSamples)));
    end
    
    % Compare residuals
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
    plot_vector_quantities_opts_shape(time_vec, residuals_1_1, [], plotopts, [2, 3], 'DisplayName', "${\rm IK}_{\rm Markers}$");
    plot_vector_quantities_opts_shape(time_vec, residuals_1_2, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm AT}$");
    plot_vector_quantities_opts_shape(time_vec, residuals_2_1, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm OPT-1}$");
    plot_vector_quantities_opts_shape(time_vec, residuals_2_2, [], [], [2, 3], 'DisplayName', "${\rm IK}_{\rm OPT-2}$");
    legend('interpreter', 'latex');
    fprintf("sum(sqrt(sum(residuals_1_1.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_1_1.^2, 2) ./ nbSamples)));
    fprintf("sum(sqrt(sum(residuals_1_2.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_1_2.^2, 2) ./ nbSamples)));
    fprintf("sum(sqrt(sum(residuals_2_1.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_2_1.^2, 2) ./ nbSamples)));
    fprintf("sum(sqrt(sum(residuals_2_2.^2, 2) ./ nbSamples)) = %.4f\n", sum(sqrt(sum(residuals_2_2.^2, 2) ./ nbSamples)));
    
    % Save the best segment lengths and joint angles
    if sum(sum(residuals_2_1.^2)) < sum(sum(residuals_2_2.^2))
        ikResults.L = L_out_2_1;
        ikResults.q = q_out_2_1;
        ikResults.residuals = residuals_2_1;
        ikResults.marker_rmse = sqrt(sum(residuals_2_1.^2, 2) ./ nbSamples);
        % Segment lengths
        figure(fig_L_time);
        plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(L_out_2_1.', [1, 2]), [], [], [2, 3], 'g', 'LineWidth', 2, 'DisplayName', '$L_{IK}$');
    else
        ikResults.L = L_out_2_2;
        ikResults.q = q_out_2_2;
        ikResults.residuals = residuals_2_2;
        ikResults.marker_rmse = sqrt(sum(residuals_2_2.^2, 2) ./ nbSamples);
        % Segment lengths
        figure(fig_L_time);
        plot_vector_quantities_opts_shape(time_vec([1, end]), repmat(L_out_2_2.', [1, 2]), [], [], [2, 3], 'g', 'LineWidth', 2, 'DisplayName', '$L_{IK}$');        
    end
    
    % Plot the lengths
    % Names of the body parts
    names = ["Shanks", "Thighs", "Pelvis-Abdomen", "Thorax", "Upper-arms", "Forearms"];
    % Data vector
    lengths = [...
    [meanL_markers];
    [defaultHumanModel.L];
    [ikResults.L];
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
    defaultHumanModel.L = ikResults.L;
    
    % Add kinematic indetification to the Structure
    Calibration.(subj).kinematicIdentification.markers_R_base = markers_R_base;
    Calibration.(subj).kinematicIdentification.markers_r_markers_base = markers_r_markers_base;
    Calibration.(subj).kinematicIdentification.TransportedMarkers = TransportedMarkers;
    Calibration.(subj).kinematicIdentification.q = ikResults.q;
    Calibration.(subj).kinematicIdentification.L = ikResults.L;
    Calibration.(subj).kinematicIdentification.residuals = ikResults.residuals;
    Calibration.(subj).kinematicIdentification.marker_rmse = ikResults.marker_rmse;
    % Add human model to structure
    Calibration.(subj).humanModel = defaultHumanModel;
    
    % Figure saving 
    figname_L_time = sprintf("%s/Segment_Lenths_in_Time.fig", subject_output_folderpath);
    saveas(fig_L_time, figname_L_time);
    figname_L = sprintf("%s/Segment_Lenths.fig", subject_output_folderpath);
    saveas(fig_L, figname_L);
    figname_kin_residuals = sprintf("%s/Kinematic_Residuals.fig", subject_output_folderpath);
    saveas(fig_kin_residuals, figname_kin_residuals);
    
    % Close all these figures
    close all;
end

%% Savename
calibration_output_filepath = sprintf("%s/%s.mat", common_output_folderpath, output_filename);
save(calibration_output_filepath, "Calibration");

%% Animate the motions

animateFlag = input("See the animations? ");

if animateFlag
    % For each subject
    for numSubj = 2 : NO_SUBJECTS

        % Subject name
        subj = sprintf("S%d", numSubj);
        % Prepare the markers structure
        % Filter the markers
        TransportedMarkers = MarkersFilter(Calibration.(subj).Markers, filt_fs, filt_cutoff, filt_order);
        % Extract position vector of the HumanModel6DOF base frame in the OptiTrack frame
        markers_r_markers_base = mean(TransportedMarkers.BODY.RANK);
        % Translate and rotate markers
        TransportedMarkers = MarkersTranslateRotate(-markers_r_markers_base, base_R_markers, TransportedMarkers);
        % 
        Calibration.(subj).kinematicIdentification.Markers = TransportedMarkers;

        % Animate
        fig_anim = figure('Position', [100, 50, 1280, 720]);
        animopts = [];
        animopts.title = sprintf("Animation of kinematically identified model alongside Markers");
        %     animopts.show_legend = true;
        %     animopts.save_path = sprintf("%s/Kinematic_Identification", subject_output_folderpath);
        Animate_nDOF_Markers(Calibration.(subj).kinematicIdentification.q,...
                             Calibration.(subj).kinematicIdentification.Markers,...
                             Calibration.(subj).kinematicIdentification.L,...
                             Calibration.(subj).SamplingTime,...
                             animopts);
        pause;
        close all;
    end
end