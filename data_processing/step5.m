%STEP5 indentifies the lifting environment for lifting trials.
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


% Kinematic identification data filepath
kinematically_calibrated_lifting_filepath = "../processed_data/Lifting/kinematically_calibrated_lifting.mat";
Lifting = importdata(kinematically_calibrated_lifting_filepath, "Lifting");

% Output folder path
common_output_folderpath = "../processed_data/Lifting";

% Output filename
output_filename = "kinematically_calibrated_lifting_with_environment";

%% Perform the computations
numSets = 4;
numReps = 5;
Trial((NO_SUBJECTS-1) * numSets * numReps) = struct();
TrialCounter = 1;

% For each subject
for numSubj = 2 : NO_SUBJECTS
% for numSubj = 2
    
    % Subject name
    subj = sprintf("S%d", numSubj);
    
    % Subject output folderpath
    subject_output_folderpath = sprintf("%s/%s", common_output_folderpath, subj);
    if ~exist(subject_output_folderpath, 'dir')
        mkdir(subject_output_folderpath);
    end
    
    % Get the ssegmentation fieldnames
    segmentationFieldnames = fields(Lifting.(subj).Segmentation);
    % Get the setnames
    setNames = segmentationFieldnames(contains(segmentationFieldnames, "Set"));
    % For each set performed by the subject
    for numSet = 1 : length(setNames)
%     for numSet = 1 : 1
        % Get the current set
        set = setNames{numSet};        
        
        % Set output folderpath
    	set_output_folderpath = sprintf("%s/%s", subject_output_folderpath, set);
        if ~exist(set_output_folderpath, 'dir')
            mkdir(set_output_folderpath);
        end
        
        % Get the indices of that set
        setIndexBegin = Lifting.(subj).Segmentation.(set).Rep1(1);
        setIndexEnd = Lifting.(subj).Segmentation.(set).Rep5(end);
        % Get the index range
        setIndexRange = setIndexBegin : setIndexEnd;
        % Get the number of samples in this iteration
        nbSamples = (setIndexEnd - setIndexBegin) + 1;        
        
        % Get the liftoff and dropoff times for all reps
        segmentation_liftoff_index = find(contains(Lifting.(subj).Segmentation.TimeLabels, "t_2"));
        t_liftoff_1 = Lifting.(subj).Segmentation.(set).Rep1(segmentation_liftoff_index);
        t_liftoff_2 = Lifting.(subj).Segmentation.(set).Rep2(segmentation_liftoff_index);
        t_liftoff_3 = Lifting.(subj).Segmentation.(set).Rep3(segmentation_liftoff_index);
        t_liftoff_4 = Lifting.(subj).Segmentation.(set).Rep4(segmentation_liftoff_index);
        t_liftoff_5 = Lifting.(subj).Segmentation.(set).Rep5(segmentation_liftoff_index);
        
        segmentation_dropoff_index = find(contains(Lifting.(subj).Segmentation.TimeLabels, "t_3"));
        t_dropoff_1 = Lifting.(subj).Segmentation.(set).Rep1(segmentation_dropoff_index);
        t_dropoff_2 = Lifting.(subj).Segmentation.(set).Rep2(segmentation_dropoff_index);
        t_dropoff_3 = Lifting.(subj).Segmentation.(set).Rep3(segmentation_dropoff_index);
        t_dropoff_4 = Lifting.(subj).Segmentation.(set).Rep4(segmentation_dropoff_index);
        t_dropoff_5 = Lifting.(subj).Segmentation.(set).Rep5(segmentation_dropoff_index);

        % Extract the forceplate ground reaction forces
        Forceplate1 = ForceplateIndex(Lifting.(subj).Forceplate, t_liftoff_1:t_dropoff_1);
        Forceplate2 = ForceplateIndex(Lifting.(subj).Forceplate, t_liftoff_2:t_dropoff_2);
        Forceplate3 = ForceplateIndex(Lifting.(subj).Forceplate, t_liftoff_3:t_dropoff_3);
        Forceplate4 = ForceplateIndex(Lifting.(subj).Forceplate, t_liftoff_4:t_dropoff_4);
        Forceplate5 = ForceplateIndex(Lifting.(subj).Forceplate, t_liftoff_5:t_dropoff_5);
        
        fp_grf_fp1 = [Forceplate1.Forces.'; Forceplate1.Moments.';];
        fp_grf_fp2 = [Forceplate2.Forces.'; Forceplate2.Moments.';];
        fp_grf_fp3 = [Forceplate3.Forces.'; Forceplate3.Moments.';];
        fp_grf_fp4 = [Forceplate4.Forces.'; Forceplate4.Moments.';];
        fp_grf_fp5 = [Forceplate5.Forces.'; Forceplate5.Moments.';];
                
        % Adapt segmentation
        Lifting.(subj).Segmentation.(set).Rep1 = Lifting.(subj).Segmentation.(set).Rep1 - Lifting.(subj).Segmentation.(set).Rep1(1)+1;
        Lifting.(subj).Segmentation.(set).Rep2 = Lifting.(subj).Segmentation.(set).Rep2 - Lifting.(subj).Segmentation.(set).Rep2(1)+1;
        Lifting.(subj).Segmentation.(set).Rep3 = Lifting.(subj).Segmentation.(set).Rep3 - Lifting.(subj).Segmentation.(set).Rep3(1)+1;
        Lifting.(subj).Segmentation.(set).Rep4 = Lifting.(subj).Segmentation.(set).Rep4 - Lifting.(subj).Segmentation.(set).Rep4(1)+1;
        Lifting.(subj).Segmentation.(set).Rep5 = Lifting.(subj).Segmentation.(set).Rep5 - Lifting.(subj).Segmentation.(set).Rep5(1)+1;
        
        % Get the liftoff and dropoff times for all reps
        segmentation_liftoff_index = find(contains(Lifting.(subj).Segmentation.TimeLabels, "t_2"));
        t_liftoff_1 = Lifting.(subj).Segmentation.(set).Rep1(segmentation_liftoff_index);
        t_liftoff_2 = Lifting.(subj).Segmentation.(set).Rep2(segmentation_liftoff_index);
        t_liftoff_3 = Lifting.(subj).Segmentation.(set).Rep3(segmentation_liftoff_index);
        t_liftoff_4 = Lifting.(subj).Segmentation.(set).Rep4(segmentation_liftoff_index);
        t_liftoff_5 = Lifting.(subj).Segmentation.(set).Rep5(segmentation_liftoff_index);
        
        segmentation_dropoff_index = find(contains(Lifting.(subj).Segmentation.TimeLabels, "t_3"));
        t_dropoff_1 = Lifting.(subj).Segmentation.(set).Rep1(segmentation_dropoff_index);
        t_dropoff_2 = Lifting.(subj).Segmentation.(set).Rep2(segmentation_dropoff_index);
        t_dropoff_3 = Lifting.(subj).Segmentation.(set).Rep3(segmentation_dropoff_index);
        t_dropoff_4 = Lifting.(subj).Segmentation.(set).Rep4(segmentation_dropoff_index);
        t_dropoff_5 = Lifting.(subj).Segmentation.(set).Rep5(segmentation_dropoff_index);
        
        % Number of samples for each rep
        nbSamples1 = (t_dropoff_1 - t_liftoff_1) + 1;
        nbSamples2 = (t_dropoff_2 - t_liftoff_2) + 1;
        nbSamples3 = (t_dropoff_3 - t_liftoff_3) + 1;
        nbSamples4 = (t_dropoff_4 - t_liftoff_4) + 1;
        nbSamples5 = (t_dropoff_5 - t_liftoff_5) + 1;
        
        % Get the time vectors for each
        time_vec_1 = 0 : Lifting.(subj).SamplingTime : (nbSamples1 - 1) * Lifting.(subj).SamplingTime;
        time_vec_2 = 0 : Lifting.(subj).SamplingTime : (nbSamples2 - 1) * Lifting.(subj).SamplingTime;
        time_vec_3 = 0 : Lifting.(subj).SamplingTime : (nbSamples3 - 1) * Lifting.(subj).SamplingTime;
        time_vec_4 = 0 : Lifting.(subj).SamplingTime : (nbSamples4 - 1) * Lifting.(subj).SamplingTime;
        time_vec_5 = 0 : Lifting.(subj).SamplingTime : (nbSamples5 - 1) * Lifting.(subj).SamplingTime;
        
        % Get the joint angles for each rep
        q1 = Lifting.(subj).(set).kinematicIdentification.q(:, t_liftoff_1:t_dropoff_1);
        q2 = Lifting.(subj).(set).kinematicIdentification.q(:, t_liftoff_2:t_dropoff_2);
        q3 = Lifting.(subj).(set).kinematicIdentification.q(:, t_liftoff_3:t_dropoff_3);
        q4 = Lifting.(subj).(set).kinematicIdentification.q(:, t_liftoff_4:t_dropoff_4);
        q5 = Lifting.(subj).(set).kinematicIdentification.q(:, t_liftoff_5:t_dropoff_5);
        
        % Get the velocities and accelerations
        dq1 = gradient(q1) ./ Lifting.(subj).SamplingTime;
        dq2 = gradient(q2) ./ Lifting.(subj).SamplingTime;
        dq3 = gradient(q3) ./ Lifting.(subj).SamplingTime;
        dq4 = gradient(q4) ./ Lifting.(subj).SamplingTime;
        dq5 = gradient(q5) ./ Lifting.(subj).SamplingTime;
        
        ddq1 = gradient(dq1) ./ Lifting.(subj).SamplingTime;
        ddq2 = gradient(dq2) ./ Lifting.(subj).SamplingTime;
        ddq3 = gradient(dq3) ./ Lifting.(subj).SamplingTime;
        ddq4 = gradient(dq4) ./ Lifting.(subj).SamplingTime;
        ddq5 = gradient(dq5) ./ Lifting.(subj).SamplingTime;
        
        % Get initial and final joint angles
        JointAnglesInitial1 = q1(:, 1);
        JointAnglesInitial2 = q2(:, 1);
        JointAnglesInitial3 = q3(:, 1);
        JointAnglesInitial4 = q4(:, 1);
        JointAnglesInitial5 = q5(:, 1);
        
        JointAnglesFinal1 = q1(:, end);
        JointAnglesFinal2 = q2(:, end);
        JointAnglesFinal3 = q3(:, end);
        JointAnglesFinal4 = q4(:, end);
        JointAnglesFinal5 = q5(:, end);
        
        % Draw the joint angle information
        supertitle = sprintf("%s : %s", subj, set);
        titles = ["$q_1$ (ANKLE)", "$q_2$ (KNEE)", "$q_3$ (HIP)", "$q_4$ (BACK)", "$q_5$ (SHOULDER)", "$q_6$ (ELBOW)"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(5);
        fig_angles_allreps = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(q1), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', 'Rep$_1$');
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(q2), [], [], [3, 2], 'Color', CMAP(2, :), 'DisplayName', 'Rep$_2$');
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(q3), [], [], [3, 2], 'Color', CMAP(3, :), 'DisplayName', 'Rep$_3$');
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(q4), [], [], [3, 2], 'Color', CMAP(4, :), 'DisplayName', 'Rep$_4$');
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(q5), [], [], [3, 2], 'Color', CMAP(5, :), 'DisplayName', 'Rep$_5$');
        legend('interpreter', 'latex', 'location', 'best');
        
        % Extract the markers for easier reference
        TransportedMarkers1 = MarkersIndex(Lifting.(subj).(set).kinematicIdentification.TransportedMarkers, t_liftoff_1:t_dropoff_1);
        TransportedMarkers2 = MarkersIndex(Lifting.(subj).(set).kinematicIdentification.TransportedMarkers, t_liftoff_2:t_dropoff_2);
        TransportedMarkers3 = MarkersIndex(Lifting.(subj).(set).kinematicIdentification.TransportedMarkers, t_liftoff_3:t_dropoff_3);
        TransportedMarkers4 = MarkersIndex(Lifting.(subj).(set).kinematicIdentification.TransportedMarkers, t_liftoff_4:t_dropoff_4);
        TransportedMarkers5 = MarkersIndex(Lifting.(subj).(set).kinematicIdentification.TransportedMarkers, t_liftoff_5:t_dropoff_5);

        % Vector from wrist to the box grip point
        WristToBoxGripPointVector1 = MarkersWristToBoxGripPoint(TransportedMarkers1);
        WristToBoxGripPointVector2 = MarkersWristToBoxGripPoint(TransportedMarkers2);
        WristToBoxGripPointVector3 = MarkersWristToBoxGripPoint(TransportedMarkers3);
        WristToBoxGripPointVector4 = MarkersWristToBoxGripPoint(TransportedMarkers4);
        WristToBoxGripPointVector5 = MarkersWristToBoxGripPoint(TransportedMarkers5);
        % Get norm
        normWristToBoxGripPointVector1 = sqrt(sum(WristToBoxGripPointVector1(:, 1:2).^2, 2));
        normWristToBoxGripPointVector2 = sqrt(sum(WristToBoxGripPointVector2(:, 1:2).^2, 2));
        normWristToBoxGripPointVector3 = sqrt(sum(WristToBoxGripPointVector3(:, 1:2).^2, 2));
        normWristToBoxGripPointVector4 = sqrt(sum(WristToBoxGripPointVector4(:, 1:2).^2, 2));
        normWristToBoxGripPointVector5 = sqrt(sum(WristToBoxGripPointVector5(:, 1:2).^2, 2));
        % Get angle
        angleWristToBoxGripPointVector1 = atan2(WristToBoxGripPointVector1(:, 2), WristToBoxGripPointVector1(:, 1));
        angleWristToBoxGripPointVector2 = atan2(WristToBoxGripPointVector2(:, 2), WristToBoxGripPointVector2(:, 1));
        angleWristToBoxGripPointVector3 = atan2(WristToBoxGripPointVector3(:, 2), WristToBoxGripPointVector3(:, 1));
        angleWristToBoxGripPointVector4 = atan2(WristToBoxGripPointVector4(:, 2), WristToBoxGripPointVector4(:, 1));
        angleWristToBoxGripPointVector5 = atan2(WristToBoxGripPointVector5(:, 2), WristToBoxGripPointVector5(:, 1));
        % Determine wrist angle
        angleDegWrist1 = rad2deg(atan2(WristToBoxGripPointVector1(:, 2), WristToBoxGripPointVector1(:, 1)) - sum(q1).');
        angleDegWrist2 = rad2deg(atan2(WristToBoxGripPointVector2(:, 2), WristToBoxGripPointVector2(:, 1)) - sum(q2).');
        angleDegWrist3 = rad2deg(atan2(WristToBoxGripPointVector3(:, 2), WristToBoxGripPointVector3(:, 1)) - sum(q3).');
        angleDegWrist4 = rad2deg(atan2(WristToBoxGripPointVector4(:, 2), WristToBoxGripPointVector4(:, 1)) - sum(q4).');
        angleDegWrist5 = rad2deg(atan2(WristToBoxGripPointVector5(:, 2), WristToBoxGripPointVector5(:, 1)) - sum(q5).');
        
        % Get mean not as cartesian mean but as mean in angle space
%         meanWristToBoxGripPointVector1 = mean(normWristToBoxGripPointVector1) * [cos(mean(angleWristToBoxGripPointVector1)); sin(mean(angleWristToBoxGripPointVector1))];
%         meanWristToBoxGripPointVector2 = mean(normWristToBoxGripPointVector2) * [cos(mean(angleWristToBoxGripPointVector2)); sin(mean(angleWristToBoxGripPointVector2))];
%         meanWristToBoxGripPointVector3 = mean(normWristToBoxGripPointVector3) * [cos(mean(angleWristToBoxGripPointVector3)); sin(mean(angleWristToBoxGripPointVector3))];
%         meanWristToBoxGripPointVector4 = mean(normWristToBoxGripPointVector4) * [cos(mean(angleWristToBoxGripPointVector4)); sin(mean(angleWristToBoxGripPointVector4))];
%         meanWristToBoxGripPointVector5 = mean(normWristToBoxGripPointVector5) * [cos(mean(angleWristToBoxGripPointVector5)); sin(mean(angleWristToBoxGripPointVector5))];
        
        meanWristToBoxGripPointVector1 = mean(WristToBoxGripPointVector1(:, 1:2));
        meanWristToBoxGripPointVector2 = mean(WristToBoxGripPointVector2(:, 1:2));
        meanWristToBoxGripPointVector3 = mean(WristToBoxGripPointVector3(:, 1:2));
        meanWristToBoxGripPointVector4 = mean(WristToBoxGripPointVector4(:, 1:2));
        meanWristToBoxGripPointVector5 = mean(WristToBoxGripPointVector5(:, 1:2));
        
        % Draw the wrist to box vector informationthem
        supertitle = sprintf("%s : %s", subj, set);
        titles = ["$\arg{r_{\rm WRI, BOX}}$", "$\|r_{\rm WRI, BOX}\|$", "$\pi^X \ r_{\rm WRI, BOX}$", "$\pi^Y \ r_{\rm WRI, BOX}$", "$\pi^Z \ r_{\rm WRI, BOX}$"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(5);
        fig_wrist_to_box_grip_point_vector = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, [angleDegWrist1.'; normWristToBoxGripPointVector1.'; WristToBoxGripPointVector1.'], [], plotopts, [5, 1], 'Color', CMAP(1, :), 'DisplayName', 'Rep$_1$');
        plot_vector_quantities_opts_shape(time_vec_2, [angleDegWrist2.'; normWristToBoxGripPointVector2.'; WristToBoxGripPointVector2.'], [], [], [5, 1], 'Color', CMAP(2, :), 'DisplayName', 'Rep$_2$');
        plot_vector_quantities_opts_shape(time_vec_3, [angleDegWrist3.'; normWristToBoxGripPointVector3.'; WristToBoxGripPointVector3.'], [], [], [5, 1], 'Color', CMAP(3, :), 'DisplayName', 'Rep$_3$');
        plot_vector_quantities_opts_shape(time_vec_4, [angleDegWrist4.'; normWristToBoxGripPointVector4.'; WristToBoxGripPointVector4.'], [], [], [5, 1], 'Color', CMAP(4, :), 'DisplayName', 'Rep$_4$');
        plot_vector_quantities_opts_shape(time_vec_5, [angleDegWrist5.'; normWristToBoxGripPointVector5.'; WristToBoxGripPointVector5.'], [], [], [5, 1], 'Color', CMAP(5, :), 'DisplayName', 'Rep$_5$');
        legend('interpreter', 'latex', 'location', 'best');
                
        % Get table centers
        [TableCenter1, TableWidth1, TableHeight1] = MarkersTableCenter(TransportedMarkers1);
        [TableCenter2, TableWidth2, TableHeight2] = MarkersTableCenter(TransportedMarkers2);
        [TableCenter3, TableWidth3, TableHeight3] = MarkersTableCenter(TransportedMarkers3);
        [TableCenter4, TableWidth4, TableHeight4] = MarkersTableCenter(TransportedMarkers4);
        [TableCenter5, TableWidth5, TableHeight5] = MarkersTableCenter(TransportedMarkers5);
        
        % Extract the human model
        humanModel = Lifting.(subj).(set).humanModel;
        
        % Get the forward kinematic propagations
        humanModel = humanModel.setDefaultKinematicPointsOfInterest();
        
        FKM1 = humanModel.forwardKinematicModel(q1);
        FKM2 = humanModel.forwardKinematicModel(q2);
        FKM3 = humanModel.forwardKinematicModel(q3);
        FKM4 = humanModel.forwardKinematicModel(q4);
        FKM5 = humanModel.forwardKinematicModel(q5);
        
        % Extract initial and final positions of the wrist/hand
        WristPositions1 = FKM1{contains([humanModel.KPOI.Name], "Hand")};
        WristPositions2 = FKM2{contains([humanModel.KPOI.Name], "Hand")};
        WristPositions3 = FKM3{contains([humanModel.KPOI.Name], "Hand")};
        WristPositions4 = FKM4{contains([humanModel.KPOI.Name], "Hand")};
        WristPositions5 = FKM5{contains([humanModel.KPOI.Name], "Hand")};
        
        WristInitialPosition1 = WristPositions1(1:2, 1);
        WristInitialPosition2 = WristPositions2(1:2, 1);
        WristInitialPosition3 = WristPositions3(1:2, 1);
        WristInitialPosition4 = WristPositions4(1:2, 1);
        WristInitialPosition5 = WristPositions5(1:2, 1);
        
        WristFinalPosition1 = WristPositions1(1:2, end);
        WristFinalPosition2 = WristPositions2(1:2, end);
        WristFinalPosition3 = WristPositions3(1:2, end);
        WristFinalPosition4 = WristPositions4(1:2, end);
        WristFinalPosition5 = WristPositions5(1:2, end);
        
        % Create lifting environments for every rep
        liftingEnvironment1 = LiftingEnvironment();
        liftingEnvironment2 = LiftingEnvironment();
        liftingEnvironment3 = LiftingEnvironment();
        liftingEnvironment4 = LiftingEnvironment();
        liftingEnvironment5 = LiftingEnvironment();
        
        % Set the lifting environment table properties
        liftingEnvironment1.TableHeight = TableHeight1;
        liftingEnvironment2.TableHeight = TableHeight2;
        liftingEnvironment3.TableHeight = TableHeight3;
        liftingEnvironment4.TableHeight = TableHeight4;
        liftingEnvironment5.TableHeight = TableHeight5;
        
        liftingEnvironment1.TableWidth = TableWidth1;
        liftingEnvironment2.TableWidth = TableWidth2;
        liftingEnvironment3.TableWidth = TableWidth3;
        liftingEnvironment4.TableWidth = TableWidth4;
        liftingEnvironment5.TableWidth = TableWidth5;
        
        liftingEnvironment1.TableCenterCoordinates = TableCenter1;
        liftingEnvironment2.TableCenterCoordinates = TableCenter2;
        liftingEnvironment3.TableCenterCoordinates = TableCenter3;
        liftingEnvironment4.TableCenterCoordinates = TableCenter4;
        liftingEnvironment5.TableCenterCoordinates = TableCenter5;
        
        % Set wrist information
        liftingEnvironment1.WristToBoxGripPointVector = meanWristToBoxGripPointVector1;
        liftingEnvironment2.WristToBoxGripPointVector = meanWristToBoxGripPointVector2;
        liftingEnvironment3.WristToBoxGripPointVector = meanWristToBoxGripPointVector3;
        liftingEnvironment4.WristToBoxGripPointVector = meanWristToBoxGripPointVector4;
        liftingEnvironment5.WristToBoxGripPointVector = meanWristToBoxGripPointVector5;
        
        liftingEnvironment1.WristInitialPosition = WristInitialPosition1;
        liftingEnvironment2.WristInitialPosition = WristInitialPosition2;
        liftingEnvironment3.WristInitialPosition = WristInitialPosition3;
        liftingEnvironment4.WristInitialPosition = WristInitialPosition4;
        liftingEnvironment5.WristInitialPosition = WristInitialPosition5;
        
        liftingEnvironment1.WristFinalPosition = WristFinalPosition1;
        liftingEnvironment2.WristFinalPosition = WristFinalPosition2;
        liftingEnvironment3.WristFinalPosition = WristFinalPosition3;
        liftingEnvironment4.WristFinalPosition = WristFinalPosition4;
        liftingEnvironment5.WristFinalPosition = WristFinalPosition5;
        
        % Set joint angle information
        liftingEnvironment1.JointAnglesInitial = JointAnglesInitial1;
        liftingEnvironment2.JointAnglesInitial = JointAnglesInitial2;
        liftingEnvironment3.JointAnglesInitial = JointAnglesInitial3;
        liftingEnvironment4.JointAnglesInitial = JointAnglesInitial4;
        liftingEnvironment5.JointAnglesInitial = JointAnglesInitial5;
        
        liftingEnvironment1.JointAnglesFinal = JointAnglesFinal1;
        liftingEnvironment2.JointAnglesFinal = JointAnglesFinal2;
        liftingEnvironment3.JointAnglesFinal = JointAnglesFinal3;
        liftingEnvironment4.JointAnglesFinal = JointAnglesFinal4;
        liftingEnvironment5.JointAnglesFinal = JointAnglesFinal5;
        
        % Get spline trajectory representations of the trajectories
        % Set the spline degree
        splineDegree = 5;
        % Set the number of knots of the spline
        numSplineKnots = 15;
        % Get the spline knot parameters
        [knotTimes1, knotValues1] = SplineTrajectory.getEquidistantSplineKnots(q1, time_vec_1, numSplineKnots);
        [knotTimes2, knotValues2] = SplineTrajectory.getEquidistantSplineKnots(q2, time_vec_2, numSplineKnots);
        [knotTimes3, knotValues3] = SplineTrajectory.getEquidistantSplineKnots(q3, time_vec_3, numSplineKnots);
        [knotTimes4, knotValues4] = SplineTrajectory.getEquidistantSplineKnots(q4, time_vec_4, numSplineKnots);
        [knotTimes5, knotValues5] = SplineTrajectory.getEquidistantSplineKnots(q5, time_vec_5, numSplineKnots);
        
        % Create spline trajectories
        splineTrajectory1 = SplineTrajectory(knotTimes1, knotValues1, splineDegree);
        splineTrajectory2 = SplineTrajectory(knotTimes2, knotValues2, splineDegree);
        splineTrajectory3 = SplineTrajectory(knotTimes3, knotValues3, splineDegree);
        splineTrajectory4 = SplineTrajectory(knotTimes4, knotValues4, splineDegree);
        splineTrajectory5 = SplineTrajectory(knotTimes5, knotValues5, splineDegree);
        
        % Evaluate spline at the time vectors
        splineTrajectory1 = splineTrajectory1.computeValuesAndStore(time_vec_1, splineDegree-1);
        splineTrajectory2 = splineTrajectory2.computeValuesAndStore(time_vec_2, splineDegree-1);
        splineTrajectory3 = splineTrajectory3.computeValuesAndStore(time_vec_3, splineDegree-1);
        splineTrajectory4 = splineTrajectory4.computeValuesAndStore(time_vec_4, splineDegree-1);
        splineTrajectory5 = splineTrajectory5.computeValuesAndStore(time_vec_5, splineDegree-1);
        
        % Get interpolated trajectories
        q1_interpolated = splineTrajectory1.currentEvaluatedValuesAndDerivatives{1};
        q2_interpolated = splineTrajectory2.currentEvaluatedValuesAndDerivatives{1};
        q3_interpolated = splineTrajectory3.currentEvaluatedValuesAndDerivatives{1};
        q4_interpolated = splineTrajectory4.currentEvaluatedValuesAndDerivatives{1};
        q5_interpolated = splineTrajectory5.currentEvaluatedValuesAndDerivatives{1};
        
        dq1_interpolated = splineTrajectory1.currentEvaluatedValuesAndDerivatives{2};
        dq2_interpolated = splineTrajectory2.currentEvaluatedValuesAndDerivatives{2};
        dq3_interpolated = splineTrajectory3.currentEvaluatedValuesAndDerivatives{2};
        dq4_interpolated = splineTrajectory4.currentEvaluatedValuesAndDerivatives{2};
        dq5_interpolated = splineTrajectory5.currentEvaluatedValuesAndDerivatives{2};
        
        ddq1_interpolated = splineTrajectory1.currentEvaluatedValuesAndDerivatives{3};
        ddq2_interpolated = splineTrajectory2.currentEvaluatedValuesAndDerivatives{3};
        ddq3_interpolated = splineTrajectory3.currentEvaluatedValuesAndDerivatives{3};
        ddq4_interpolated = splineTrajectory4.currentEvaluatedValuesAndDerivatives{3};
        ddq5_interpolated = splineTrajectory5.currentEvaluatedValuesAndDerivatives{3};
        
        % Get external wrenches
        fFOOT1 = zeros(6, nbSamples1);
        fFOOT2 = zeros(6, nbSamples2);
        fFOOT3 = zeros(6, nbSamples3);
        fFOOT4 = zeros(6, nbSamples4);
        fFOOT5 = zeros(6, nbSamples5);
        
%         fHAND1 = liftingEnvironment1.getExternalWrenches(q1, humanModel.Gravity);
%         fHAND2 = liftingEnvironment2.getExternalWrenches(q2, humanModel.Gravity);
%         fHAND3 = liftingEnvironment3.getExternalWrenches(q3, humanModel.Gravity);
%         fHAND4 = liftingEnvironment4.getExternalWrenches(q4, humanModel.Gravity);
%         fHAND5 = liftingEnvironment5.getExternalWrenches(q5, humanModel.Gravity);
        
        fHAND1 = liftingEnvironment1.getExternalWrenches([q1; angleWristToBoxGripPointVector1.'], humanModel.Gravity);
        fHAND2 = liftingEnvironment2.getExternalWrenches([q2; angleWristToBoxGripPointVector2.'], humanModel.Gravity);
        fHAND3 = liftingEnvironment3.getExternalWrenches([q3; angleWristToBoxGripPointVector3.'], humanModel.Gravity);
        fHAND4 = liftingEnvironment4.getExternalWrenches([q4; angleWristToBoxGripPointVector4.'], humanModel.Gravity);
        fHAND5 = liftingEnvironment5.getExternalWrenches([q5; angleWristToBoxGripPointVector5.'], humanModel.Gravity);
        
%         fHAND1_interpolated = liftingEnvironment1.getExternalWrenches(q1_interpolated, humanModel.Gravity);
%         fHAND2_interpolated = liftingEnvironment2.getExternalWrenches(q2_interpolated, humanModel.Gravity);
%         fHAND3_interpolated = liftingEnvironment3.getExternalWrenches(q3_interpolated, humanModel.Gravity);
%         fHAND4_interpolated = liftingEnvironment4.getExternalWrenches(q4_interpolated, humanModel.Gravity);
%         fHAND5_interpolated = liftingEnvironment5.getExternalWrenches(q5_interpolated, humanModel.Gravity);
        
        fHAND1_interpolated = liftingEnvironment1.getExternalWrenches([q1_interpolated; angleWristToBoxGripPointVector1.'], humanModel.Gravity);
        fHAND2_interpolated = liftingEnvironment2.getExternalWrenches([q2_interpolated; angleWristToBoxGripPointVector2.'], humanModel.Gravity);
        fHAND3_interpolated = liftingEnvironment3.getExternalWrenches([q3_interpolated; angleWristToBoxGripPointVector3.'], humanModel.Gravity);
        fHAND4_interpolated = liftingEnvironment4.getExternalWrenches([q4_interpolated; angleWristToBoxGripPointVector4.'], humanModel.Gravity);
        fHAND5_interpolated = liftingEnvironment5.getExternalWrenches([q5_interpolated; angleWristToBoxGripPointVector5.'], humanModel.Gravity);
        
        % Get torques
        [tau1, base_grf_model1] = humanModel.inverseDynamicModel(q1, dq1, ddq1, fFOOT1, fHAND1);
        [tau2, base_grf_model2] = humanModel.inverseDynamicModel(q2, dq2, ddq2, fFOOT2, fHAND2);
        [tau3, base_grf_model3] = humanModel.inverseDynamicModel(q3, dq3, ddq3, fFOOT3, fHAND3);
        [tau4, base_grf_model4] = humanModel.inverseDynamicModel(q4, dq4, ddq4, fFOOT4, fHAND4);
        [tau5, base_grf_model5] = humanModel.inverseDynamicModel(q5, dq5, ddq5, fFOOT5, fHAND5);
        
        [tau1_interpolated, base_grf_model1_interpolated] = humanModel.inverseDynamicModel(q1_interpolated, dq1_interpolated, ddq1_interpolated, fFOOT1, fHAND1_interpolated);
        [tau2_interpolated, base_grf_model2_interpolated] = humanModel.inverseDynamicModel(q2_interpolated, dq2_interpolated, ddq2_interpolated, fFOOT2, fHAND2_interpolated);
        [tau3_interpolated, base_grf_model3_interpolated] = humanModel.inverseDynamicModel(q3_interpolated, dq3_interpolated, ddq3_interpolated, fFOOT3, fHAND3_interpolated);
        [tau4_interpolated, base_grf_model4_interpolated] = humanModel.inverseDynamicModel(q4_interpolated, dq4_interpolated, ddq4_interpolated, fFOOT4, fHAND4_interpolated);
        [tau5_interpolated, base_grf_model5_interpolated] = humanModel.inverseDynamicModel(q5_interpolated, dq5_interpolated, ddq5_interpolated, fFOOT5, fHAND5_interpolated);
        
        % Transform GRFs to marker frame
        % Extract transformation parameters
        markers_R_base = Lifting.(subj).(set).kinematicIdentification.markers_R_base;
        markers_r_markers_base = Lifting.(subj).(set).kinematicIdentification.markers_r_markers_base;
        % Transform model GRFs
        markers_grf_model1 = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model1);
        markers_grf_model2 = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model2);
        markers_grf_model3 = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model3);
        markers_grf_model4 = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model4);
        markers_grf_model5 = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model5);
        
        markers_grf_model1_interpolated = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model1_interpolated);
        markers_grf_model2_interpolated = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model2_interpolated);
        markers_grf_model3_interpolated = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model3_interpolated);
        markers_grf_model4_interpolated = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model4_interpolated);
        markers_grf_model5_interpolated = WrenchesRotateTranslate(markers_R_base, markers_r_markers_base, base_grf_model5_interpolated);
        % Transform forceplate GRFs
        markers_grf_fp1 = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp1);
        markers_grf_fp2 = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp2);
        markers_grf_fp3 = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp3);
        markers_grf_fp4 = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp4);
        markers_grf_fp5 = WrenchesRotateTranslate(markers_R_fp, markers_r_markers_fp, fp_grf_fp5);
        
        % Compare graphically joint angles
        titles = ["$q_1$ (ANKLE)", "$q_2$ (KNEE)", "$q_3$ (HIP)", "$q_4$ (BACK)", "$q_5$ (SHOULDER)", "$q_6$ (ELBOW)"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(2);
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep1");
        fig_angle_interpolation_rep1 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(q1), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$q$');
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(q1_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$q_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep2");
        fig_angle_interpolation_rep2 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(q2), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$q$');
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(q2_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$q_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep3");
        fig_angle_interpolation_rep3 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(q3), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$q$');
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(q3_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$q_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep4");
        fig_angle_interpolation_rep4 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(q4), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$q$');
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(q4_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$q_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep5");
        fig_angle_interpolation_rep5 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(q5), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$q$');
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(q5_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$q_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        
        
        % Compare graphically velocities
        titles = ["$\partial q_1 / \partial t$ (ANKLE)", "$\partial q_2 / \partial t$  (KNEE)", "$\partial q_3 / \partial t$ (HIP)",...
                  "$\partial q_4 / \partial t$  (BACK)", "$\partial q_5 / \partial t$  (SHOULDER)", "$\partial q_6 / \partial t$  (ELBOW)"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(2);
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep1");
        fig_velocity_interpolation_rep1 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(dq1), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial q / \partial t$ ');
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(dq1_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial q_{\rm interpolated}  / \partial t$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep2");
        fig_velocity_interpolation_rep2 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(dq2), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial q / \partial t$ ');
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(dq2_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial q_{\rm interpolated}  / \partial t$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep3");
        fig_velocity_interpolation_rep3 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(dq3), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial q / \partial t$ ');
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(dq3_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial q_{\rm interpolated}  / \partial t$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep4");
        fig_velocity_interpolation_rep4 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(dq4), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial q / \partial t$ ');
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(dq4_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial q_{\rm interpolated}  / \partial t$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep5");
        fig_velocity_interpolation_rep5 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(dq5), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial q / \partial t$ ');
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(dq5_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial q_{\rm interpolated}  / \partial t$');
        legend('interpreter', 'latex', 'location', 'best');
        
        
        % Compare graphically accelerations
        titles = ["$\partial^2 q_1 / \partial t^2$ (ANKLE)", "$\partial^2 q_2 / \partial t^2$  (KNEE)", "$\partial^2 q_3 / \partial t^2$ (HIP)",...
                  "$\partial^2 q_4 / \partial t^2$  (BACK)", "$\partial^2 q_5 / \partial t^2$  (SHOULDER)", "$\partial^2 q_6 / \partial t^2$  (ELBOW)"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(2);
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep1");
        fig_acceleration_interpolation_rep1 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(ddq1), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial^2 q / \partial t^2$ ');
        plot_vector_quantities_opts_shape(time_vec_1, rad2deg(ddq1_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial^2 q_{\rm interpolated}  / \partial t^2$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep2");
        fig_acceleration_interpolation_rep2 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(ddq2), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial^2 q / \partial t^2$ ');
        plot_vector_quantities_opts_shape(time_vec_2, rad2deg(ddq2_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial^2 q_{\rm interpolated}  / \partial t^2$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep3");
        fig_acceleration_interpolation_rep3 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(ddq3), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial^2 q / \partial t^2$ ');
        plot_vector_quantities_opts_shape(time_vec_3, rad2deg(ddq3_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial^2 q_{\rm interpolated}  / \partial t^2$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep4");
        fig_acceleration_interpolation_rep4 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(ddq4), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial^2 q / \partial t^2$ ');
        plot_vector_quantities_opts_shape(time_vec_4, rad2deg(ddq4_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial^2 q_{\rm interpolated}  / \partial t^2$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep5");
        fig_acceleration_interpolation_rep5 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(ddq5), [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\partial^2 q / \partial t^2$ ');
        plot_vector_quantities_opts_shape(time_vec_5, rad2deg(ddq5_interpolated), [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\partial^2 q_{\rm interpolated}  / \partial t^2$');
        legend('interpreter', 'latex', 'location', 'best');
        
        
        
        % Compare graphically joint torques
        titles = ["$\tau_1$ (ANKLE)", "$\tau_2$ (KNEE)", "$\tau_3$ (HIP)", "$\tau_4$ (BACK)", "$\tau_5$ (SHOULDER)", "$\tau_6$ (ELBOW)"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(2);
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep1");
        fig_torque_interpolation_rep1 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, tau1, [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\tau$');
        plot_vector_quantities_opts_shape(time_vec_1, tau1_interpolated, [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\tau_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep2");
        fig_torque_interpolation_rep2 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_2, tau2, [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\tau$');
        plot_vector_quantities_opts_shape(time_vec_2, tau2_interpolated, [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\tau_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep3");
        fig_torque_interpolation_rep3 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_3, tau3, [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\tau$');
        plot_vector_quantities_opts_shape(time_vec_3, tau3_interpolated, [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\tau_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep4");
        fig_torque_interpolation_rep4 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_4, tau4, [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\tau$');
        plot_vector_quantities_opts_shape(time_vec_4, tau4_interpolated, [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\tau_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep5");
        fig_torque_interpolation_rep5 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_5, tau5, [], plotopts, [3, 2], 'Color', CMAP(1, :), 'DisplayName', '$\tau$');
        plot_vector_quantities_opts_shape(time_vec_5, tau5_interpolated, [], plotopts, [3, 2], 'Color', CMAP(2, :), 'DisplayName', '$\tau_{\rm interpolated}$');
        legend('interpreter', 'latex', 'location', 'best');
        
        
        
        % Plot GRFs
        titles = ["$f_x$", "$f_y$", "$m_z$"];
        plotopts = [];
        plotopts.title = @(n) {titles(n), "interpreter", "latex"};
        plotopts.xlabel = @(n) {"time [$s$]", "interpreter", "latex"};
        CMAP = linspecer(3);
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep1");
        fig_grf_interpolation_rep1 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_1, [markers_grf_fp1(1:2, :); markers_grf_fp1(6, :)], [], plotopts, [3, 1], 'Color', CMAP(1, :), 'DisplayName', 'Forceplate');
        plot_vector_quantities_opts_shape(time_vec_1, [markers_grf_model1(1:2, :); markers_grf_model1(6, :)], [], plotopts, [3, 1], 'Color', CMAP(2, :), 'DisplayName', 'Numerical');
        plot_vector_quantities_opts_shape(time_vec_1, [markers_grf_model1_interpolated(1:2, :); markers_grf_model1_interpolated(6, :)], [], plotopts, [3, 1], 'Color', CMAP(3, :), 'DisplayName', 'Interpolated');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep2");
        fig_grf_interpolation_rep2 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_2, [markers_grf_fp2(1:2, :); markers_grf_fp2(6, :)], [], plotopts, [3, 1], 'Color', CMAP(1, :), 'DisplayName', 'Forceplate');
        plot_vector_quantities_opts_shape(time_vec_2, [markers_grf_model2(1:2, :); markers_grf_model2(6, :)], [], plotopts, [3, 1], 'Color', CMAP(2, :), 'DisplayName', 'Numerical');
        plot_vector_quantities_opts_shape(time_vec_2, [markers_grf_model2_interpolated(1:2, :); markers_grf_model2_interpolated(6, :)], [], plotopts, [3, 1], 'Color', CMAP(3, :), 'DisplayName', 'Interpolated');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep3");
        fig_grf_interpolation_rep3 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_3, [markers_grf_fp3(1:2, :); markers_grf_fp3(6, :)], [], plotopts, [3, 1], 'Color', CMAP(1, :), 'DisplayName', 'Forceplate');
        plot_vector_quantities_opts_shape(time_vec_3, [markers_grf_model3(1:2, :); markers_grf_model3(6, :)], [], plotopts, [3, 1], 'Color', CMAP(2, :), 'DisplayName', 'Numerical');
        plot_vector_quantities_opts_shape(time_vec_3, [markers_grf_model3_interpolated(1:2, :); markers_grf_model3_interpolated(6, :)], [], plotopts, [3, 1], 'Color', CMAP(3, :), 'DisplayName', 'Interpolated');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep4");
        fig_grf_interpolation_rep4 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_4, [markers_grf_fp4(1:2, :); markers_grf_fp4(6, :)], [], plotopts, [3, 1], 'Color', CMAP(1, :), 'DisplayName', 'Forceplate');
        plot_vector_quantities_opts_shape(time_vec_4, [markers_grf_model4(1:2, :); markers_grf_model4(6, :)], [], plotopts, [3, 1], 'Color', CMAP(2, :), 'DisplayName', 'Numerical');
        plot_vector_quantities_opts_shape(time_vec_4, [markers_grf_model4_interpolated(1:2, :); markers_grf_model4_interpolated(6, :)], [], plotopts, [3, 1], 'Color', CMAP(3, :), 'DisplayName', 'Interpolated');
        legend('interpreter', 'latex', 'location', 'best');
        
        supertitle = sprintf("%s : %s : %s", subj, set, "Rep5");
        fig_grf_interpolation_rep5 = figure;
        sgtitle(supertitle, "interpreter", "latex");
        hold on;
        plot_vector_quantities_opts_shape(time_vec_5, [markers_grf_fp5(1:2, :); markers_grf_fp5(6, :)], [], plotopts, [3, 1], 'Color', CMAP(1, :), 'DisplayName', 'Forceplate');
        plot_vector_quantities_opts_shape(time_vec_5, [markers_grf_model5(1:2, :); markers_grf_model5(6, :)], [], plotopts, [3, 1], 'Color', CMAP(2, :), 'DisplayName', 'Numerical');
        plot_vector_quantities_opts_shape(time_vec_5, [markers_grf_model5_interpolated(1:2, :); markers_grf_model5_interpolated(6, :)], [], plotopts, [3, 1], 'Color', CMAP(3, :), 'DisplayName', 'Interpolated');
        legend('interpreter', 'latex', 'location', 'best');
        
        % Save figures
        fig_angles_allreps_savepath = sprintf("%s/angles_all_reps.jpg", set_output_folderpath);
        saveas(fig_angles_allreps, fig_angles_allreps_savepath);
        fig_wrist_to_box_grip_point_vector_savepath = sprintf("%s/wrist_to_box_grip_point_vector_all_reps.jpg", set_output_folderpath);
        saveas(fig_wrist_to_box_grip_point_vector, fig_wrist_to_box_grip_point_vector_savepath);
        
        fig_angle_interpolation_rep1_savepath = sprintf("%s/angle_interpolation_rep1.jpg", set_output_folderpath);
        saveas(fig_angle_interpolation_rep1, fig_angle_interpolation_rep1_savepath);
        fig_angle_interpolation_rep2_savepath = sprintf("%s/angle_interpolation_rep2.jpg", set_output_folderpath);
        saveas(fig_angle_interpolation_rep2, fig_angle_interpolation_rep2_savepath);
        fig_angle_interpolation_rep3_savepath = sprintf("%s/angle_interpolation_rep3.jpg", set_output_folderpath);
        saveas(fig_angle_interpolation_rep3, fig_angle_interpolation_rep3_savepath);
        fig_angle_interpolation_rep4_savepath = sprintf("%s/angle_interpolation_rep4.jpg", set_output_folderpath);
        saveas(fig_angle_interpolation_rep4, fig_angle_interpolation_rep4_savepath);
        fig_angle_interpolation_rep5_savepath = sprintf("%s/angle_interpolation_rep5.jpg", set_output_folderpath);
        saveas(fig_angle_interpolation_rep5, fig_angle_interpolation_rep5_savepath);
        
        fig_velocity_interpolation_rep1_savepath = sprintf("%s/velocity_interpolation_rep1.jpg", set_output_folderpath);
        saveas(fig_velocity_interpolation_rep1, fig_velocity_interpolation_rep1_savepath);
        fig_velocity_interpolation_rep2_savepath = sprintf("%s/velocity_interpolation_rep2.jpg", set_output_folderpath);
        saveas(fig_velocity_interpolation_rep2, fig_velocity_interpolation_rep2_savepath);
        fig_velocity_interpolation_rep3_savepath = sprintf("%s/velocity_interpolation_rep3.jpg", set_output_folderpath);
        saveas(fig_velocity_interpolation_rep3, fig_velocity_interpolation_rep3_savepath);
        fig_velocity_interpolation_rep4_savepath = sprintf("%s/velocity_interpolation_rep4.jpg", set_output_folderpath);
        saveas(fig_velocity_interpolation_rep4, fig_velocity_interpolation_rep4_savepath);
        fig_velocity_interpolation_rep5_savepath = sprintf("%s/velocity_interpolation_rep5.jpg", set_output_folderpath);
        saveas(fig_velocity_interpolation_rep5, fig_velocity_interpolation_rep5_savepath);
        
        fig_acceleration_interpolation_rep1_savepath = sprintf("%s/acceleration_interpolation_rep1.jpg", set_output_folderpath);
        saveas(fig_acceleration_interpolation_rep1, fig_acceleration_interpolation_rep1_savepath);
        fig_acceleration_interpolation_rep2_savepath = sprintf("%s/acceleration_interpolation_rep2.jpg", set_output_folderpath);
        saveas(fig_acceleration_interpolation_rep2, fig_acceleration_interpolation_rep2_savepath);
        fig_acceleration_interpolation_rep3_savepath = sprintf("%s/acceleration_interpolation_rep3.jpg", set_output_folderpath);
        saveas(fig_acceleration_interpolation_rep3, fig_acceleration_interpolation_rep3_savepath);
        fig_acceleration_interpolation_rep4_savepath = sprintf("%s/acceleration_interpolation_rep4.jpg", set_output_folderpath);
        saveas(fig_acceleration_interpolation_rep4, fig_acceleration_interpolation_rep4_savepath);
        fig_acceleration_interpolation_rep5_savepath = sprintf("%s/acceleration_interpolation_rep5.jpg", set_output_folderpath);
        saveas(fig_acceleration_interpolation_rep5, fig_acceleration_interpolation_rep5_savepath);
        
        fig_torque_interpolation_rep1_savepath = sprintf("%s/torque_interpolation_rep1.jpg", set_output_folderpath);
        saveas(fig_torque_interpolation_rep1, fig_torque_interpolation_rep1_savepath);
        fig_torque_interpolation_rep2_savepath = sprintf("%s/torque_interpolation_rep2.jpg", set_output_folderpath);
        saveas(fig_torque_interpolation_rep2, fig_torque_interpolation_rep2_savepath);
        fig_torque_interpolation_rep3_savepath = sprintf("%s/torque_interpolation_rep3.jpg", set_output_folderpath);
        saveas(fig_torque_interpolation_rep3, fig_torque_interpolation_rep3_savepath);
        fig_torque_interpolation_rep4_savepath = sprintf("%s/torque_interpolation_rep4.jpg", set_output_folderpath);
        saveas(fig_torque_interpolation_rep4, fig_torque_interpolation_rep4_savepath);
        fig_torque_interpolation_rep5_savepath = sprintf("%s/torque_interpolation_rep5.jpg", set_output_folderpath);
        saveas(fig_torque_interpolation_rep5, fig_torque_interpolation_rep5_savepath);
        
        fig_grf_interpolation_rep1_savepath = sprintf("%s/grf_interpolation_rep1.jpg", set_output_folderpath);
        saveas(fig_grf_interpolation_rep1, fig_grf_interpolation_rep1_savepath);
        fig_grf_interpolation_rep2_savepath = sprintf("%s/grf_interpolation_rep2.jpg", set_output_folderpath);
        saveas(fig_grf_interpolation_rep2, fig_grf_interpolation_rep2_savepath);
        fig_grf_interpolation_rep3_savepath = sprintf("%s/grf_interpolation_rep3.jpg", set_output_folderpath);
        saveas(fig_grf_interpolation_rep3, fig_grf_interpolation_rep3_savepath);
        fig_grf_interpolation_rep4_savepath = sprintf("%s/grf_interpolation_rep4.jpg", set_output_folderpath);
        saveas(fig_grf_interpolation_rep4, fig_grf_interpolation_rep4_savepath);
        fig_grf_interpolation_rep5_savepath = sprintf("%s/grf_interpolation_rep5.jpg", set_output_folderpath);
        saveas(fig_grf_interpolation_rep5, fig_grf_interpolation_rep5_savepath);
        
        % Close all figures after saving
        close all;
        
        % Update trials structure
        % Rep 1
        Trials(TrialCounter).subject = subj;
        Trials(TrialCounter).set = set;
        Trials(TrialCounter).rep = "Rep1";
        Trials(TrialCounter).nbSamples = nbSamples1;
        Trials(TrialCounter).t = time_vec_1;
        Trials(TrialCounter).q = q1;
        Trials(TrialCounter).dq = dq1;
        Trials(TrialCounter).ddq = ddq1;
        Trials(TrialCounter).Markers = TransportedMarkers1;
        Trials(TrialCounter).Forceplate = Forceplate1;
        Trials(TrialCounter).splineTrajectory = splineTrajectory1;
        Trials(TrialCounter).liftingEnvironment = liftingEnvironment1;
        Trials(TrialCounter).humanModel = humanModel;
        
        TrialCounter = TrialCounter + 1;
        
        % Rep 2
        Trials(TrialCounter).subject = subj;
        Trials(TrialCounter).set = set;
        Trials(TrialCounter).rep = "Rep2";
        Trials(TrialCounter).nbSamples = nbSamples2;
        Trials(TrialCounter).t = time_vec_2;
        Trials(TrialCounter).q = q2;
        Trials(TrialCounter).dq = dq2;
        Trials(TrialCounter).ddq = ddq2;
        Trials(TrialCounter).Markers = TransportedMarkers2;
        Trials(TrialCounter).Forceplate = Forceplate2;
        Trials(TrialCounter).splineTrajectory = splineTrajectory2;
        Trials(TrialCounter).liftingEnvironment = liftingEnvironment2;
        Trials(TrialCounter).humanModel = humanModel;
        
        TrialCounter = TrialCounter + 1;
        
        % Rep 3
        Trials(TrialCounter).subject = subj;
        Trials(TrialCounter).set = set;
        Trials(TrialCounter).rep = "Rep3";
        Trials(TrialCounter).nbSamples = nbSamples3;
        Trials(TrialCounter).t = time_vec_3;
        Trials(TrialCounter).q = q3;
        Trials(TrialCounter).dq = dq3;
        Trials(TrialCounter).ddq = ddq3;
        Trials(TrialCounter).Markers = TransportedMarkers3;
        Trials(TrialCounter).Forceplate = Forceplate3;
        Trials(TrialCounter).splineTrajectory = splineTrajectory3;
        Trials(TrialCounter).liftingEnvironment = liftingEnvironment3;
        Trials(TrialCounter).humanModel = humanModel;
        
        TrialCounter = TrialCounter + 1;
        
        % Rep 4
        Trials(TrialCounter).subject = subj;
        Trials(TrialCounter).set = set;
        Trials(TrialCounter).rep = "Rep4";
        Trials(TrialCounter).nbSamples = nbSamples4;
        Trials(TrialCounter).t = time_vec_4;
        Trials(TrialCounter).q = q4;
        Trials(TrialCounter).dq = dq4;
        Trials(TrialCounter).ddq = ddq4;
        Trials(TrialCounter).Markers = TransportedMarkers4;
        Trials(TrialCounter).Forceplate = Forceplate4;
        Trials(TrialCounter).splineTrajectory = splineTrajectory4;
        Trials(TrialCounter).liftingEnvironment = liftingEnvironment4;
        Trials(TrialCounter).humanModel = humanModel;
        
        TrialCounter = TrialCounter + 1;
        
        % Rep 5
        Trials(TrialCounter).subject = subj;
        Trials(TrialCounter).set = set;
        Trials(TrialCounter).rep = "Rep5";
        Trials(TrialCounter).nbSamples = nbSamples5;
        Trials(TrialCounter).t = time_vec_5;
        Trials(TrialCounter).q = q5;
        Trials(TrialCounter).dq = dq5;
        Trials(TrialCounter).ddq = ddq5;
        Trials(TrialCounter).Markers = TransportedMarkers5;
        Trials(TrialCounter).Forceplate = Forceplate5;
        Trials(TrialCounter).splineTrajectory = splineTrajectory5;
        Trials(TrialCounter).liftingEnvironment = liftingEnvironment5;
        Trials(TrialCounter).humanModel = humanModel;
        
        TrialCounter = TrialCounter + 1;
        
    end
end

kinematically_calibrated_lifting_with_environment_filepath = sprintf("%s/%s.mat", common_output_folderpath, output_filename);
save(kinematically_calibrated_lifting_with_environment_filepath, "Trials");