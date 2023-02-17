%STEP0 is a script that transforms all the csv files within the raw_data
%directory to .mat files.
%The first part of the script:
%   - Defines all the filepaths of the raw data that need to be loaded. Raw
%   data includes Forceplate data, OptiTrack Markers data, and manual
%   segmentation data.
%   - Loads all the data into three different structures, depending on what
%   the data corresponds to. The structures are:
%       - Calibration: For data corresponding to the calibration
%       experiment, includes Forceplate and Markers data.
%       - Lifting: For data corresponding to the normal lifting experiment,
%       which includes includes Forceplate and Markers data, alongside the
%       manual segmentation data.
%       - SquatLifting: For data corresponding to the squat lifting
%       experiment, which includes Forceplate and Markers data, alongside 
%       the manual segmentation data.
%   Each of these structures contains substructures corresponding to the
%   subjects. These substructures are named S1, S2, ..., S7.
%   The substructures S1, S2, ..., S7 contain substructures Forceplate,
%   Markers, as well as the fields AGE, HEIGHT and WEIGHT. The Forceplate
%   substructure contains the fields:
%       - Forces: For the forces measured by the forceplate during the
%       experiment in the forceplate local frame.
%       - Moments: For the moments measured by the forceplate during the
%       experiment in the forceplate local frame.
%       - COP: For the Center of Pressure position measured by the 
%       forceplate during the experiment in the forceplate local frame.
%   The Markers substrucutre contains the substructures:
%       - BODY: which contains fields, named after the individual Marker
%       names (e.g. RANK for right ankle, RWRI for right wrist, ...), which
%       themselves contain the Marker X,Y,Z positions measured by the
%       OptiTrack system in the OptiTrack reference frame.
%       - BOX: which contains fields, named after the individual Marker
%       names (e.g. FARDL for far down left, NEADR for near down right, ...),
%       which themselves contain the Marker X,Y,Z positions measured by the
%       OptiTrack system in the OptiTrack reference frame.
%       - TABLE: which contains fields, named after the individual Marker
%       names (e.g. FARL for far left, NEAR for near right, ...), which
%       themselves contain the Marker X,Y,Z positions measured by the
%       OptiTrack system in the OptiTrack reference frame.
%   The AGE, HEIGHT, and WEIGHT fields contain information about the 
%   subject, namely the age, height, and weight.
%   
%   The Lifting S1, S2, ..., S7 substructures contain the following fields:
%       -segmentationTimeLabels: contains the labels given to the
%       segmentation times, which are explained in the segmentation files.
%       These are meant for use during later programming.
%       -segmentationTimes_Set1: segmentation of the first set of 5
%       repetitions in the lifting experiment.
%       -segmentationTimes_Set2: segmentation of the second set of 5
%       repetitions in the lifting experiment.
%       -segmentationTimes_Set3: segmentation of the third set of 5
%       repetitions in the lifting experiment.
%       -segmentationTimes_Set4: segmentation of the fourth set of 5
%       repetitions in the lifting experiment.
%   The SquatLifting S1, S2, ..., S7 substructures contain the following 
%   fields:
%       -segmentationTimeLabels: contains the labels given to the
%       segmentation times, which are explained in the segmentation files.
%       These are meant for use during later programming.
%       -segmentationTimes_Set1: segmentation of the first and only set of 
%       5 repetitions in the squat lifting experiment.
%The second part of the script:
%   - Fills gaps in Markers using the FillCubic function
%   - Rotates the Marker coordinates so that the X-axis is pointing in the
%   posterior-anterior axis, while the Z-axis is pointing along the
%   medio-lateral axis from left to right.
%   - Undersamples Forceplate data if it has more samples than the Markers
%   (as we've had some data that was recorded with 1000Hz on the Forceplate
%   and 100Hz on the Optitrack)
%The third part of the script:
%   - Saves the Calibration, Lifting and SquatLifting structures inside
%   their own .mat files within the processed_data directory.

clear all;
close all;
clc;

% Define constants
forceplate_filepaths = [ ...
"../raw_data/S1/Calibration_Forceplate.csv";
"../raw_data/S1/Lifting_Forceplate.csv";
"../raw_data/S1/Squatlifting_Forceplate.csv";
"../raw_data/S2/Calibration_Forceplate.csv";
"../raw_data/S2/Lifting_Forceplate.csv";
"../raw_data/S2/Squatlifting_Forceplate.csv";
"../raw_data/S3/Calibration_Forceplate.csv";
"../raw_data/S3/Lifting_Forceplate.csv";
"../raw_data/S3/Squatlifting1_Forceplate.csv";
"../raw_data/S3/Squatlifting2_Forceplate.csv";
"../raw_data/S4/Calibration_Forceplate.csv";
"../raw_data/S4/Lifting_Forceplate.csv";
"../raw_data/S4/Squatlifting_Forceplate.csv";
"../raw_data/S5/Calibration_Forceplate.csv";
"../raw_data/S5/Lifting_Forceplate.csv";
"../raw_data/S5/Squatlifting_Forceplate.csv";
"../raw_data/S6/Calibration_Forceplate.csv";
"../raw_data/S6/Lifting_Forceplate.csv";
"../raw_data/S6/Squatlifting_Forceplate.csv";
"../raw_data/S7/Calibration_Forceplate.csv";
"../raw_data/S7/Lifting_Forceplate.csv";
"../raw_data/S7/Squatlifting_Forceplate.csv";
];

markers_filepaths = [ ...
"../raw_data/S1/Calibration_Markers.csv";
"../raw_data/S1/Lifting_Markers.csv";
"../raw_data/S1/Squatlifting_Markers.csv";
"../raw_data/S2/Calibration_Markers.csv";
"../raw_data/S2/Lifting_Markers.csv";
"../raw_data/S2/Squatlifting_Markers.csv";
"../raw_data/S3/Calibration_Markers.csv";
"../raw_data/S3/Lifting_Markers.csv";
"../raw_data/S3/Squatlifting1_Markers.csv";
"../raw_data/S3/Squatlifting2_Markers.csv";
"../raw_data/S4/Calibration_Markers.csv";
"../raw_data/S4/Lifting_Markers.csv";
"../raw_data/S4/Squatlifting_Markers.csv";
"../raw_data/S5/Calibration_Markers.csv";
"../raw_data/S5/Lifting_Markers.csv";
"../raw_data/S5/Squatlifting_Markers.csv";
"../raw_data/S6/Calibration_Markers.csv";
"../raw_data/S6/Lifting_Markers.csv";
"../raw_data/S6/Squatlifting_Markers.csv";
"../raw_data/S7/Calibration_Markers.csv";
"../raw_data/S7/Lifting_Markers.csv";
"../raw_data/S7/Squatlifting_Markers.csv";
];

segmentation_filepaths = [...
"../raw_data/S1/Lifting_Segmentation.xlsx";
"../raw_data/S1/Squatlifting_Segmentation.xlsx";
"../raw_data/S2/Lifting_Segmentation.xlsx";
"../raw_data/S2/Squatlifting_Segmentation.xlsx";
"../raw_data/S3/Lifting_Segmentation.xlsx";
"../raw_data/S3/Squatlifting1_Segmentation.xlsx";
"../raw_data/S3/Squatlifting2_Segmentation.xlsx";
"../raw_data/S4/Lifting_Segmentation.xlsx";
"../raw_data/S4/Squatlifting_Segmentation.xlsx";
"../raw_data/S5/Lifting_Segmentation.xlsx";
"../raw_data/S5/Squatlifting_Segmentation.xlsx";
"../raw_data/S6/Lifting_Segmentation.xlsx";
"../raw_data/S6/Squatlifting_Segmentation.xlsx";
"../raw_data/S7/Lifting_Segmentation.xlsx";
"../raw_data/S7/Squatlifting_Segmentation.xlsx";
];

subject_info_filepath = "../raw_data/subjects_info.xlsx";

% Sampling rate
SamplingTime = 0.01;
SamplingFrequency = 100;

% General info
NO_SUBJECTS = 7;

% Define the rotation matrix from the OptiTrack frame to the Body Frame
base_R_markers = roty(180);

% Forceplate data storage
FP_1st_DATA_ROW = 33;
FP_FORCE_COLS = 4:6;
FP_MOMENT_COLS = 7:9;
FP_COP_COLS = 10:12;

% Markers data storage
MK_1st_DATA_ROW = 7;
MK_NAMES_ROW = 3;
MK_COORDINATE_ROW = 6;
MK_MKSET_SEMICOL = ':';

% Segmentation data storage
SG_LIFTING_SET1_ROWS = (32:36);
SG_LIFTING_SET2_ROWS = (39:43);
SG_LIFTING_SET3_ROWS = (46:50);
SG_LIFTING_SET4_ROWS = (53:57);
SG_SQUATLIFTING_SET1_ROWS = (33:37);
SG_TIME_COLS = 5:16;
SG_TIME_LABELS = [...
"t_lift";
"t_1";
"t_2";
"t_3";
"t_4";
"t_lift_end";
"t_lower";
"t_5";
"t_6";
"t_7";
"t_8";
"t_lower_end";
].';

% Subject info storage constants
SI_CM_TO_M = @(h) h / 100;
SI_AGE_COLUMN = 3;
SI_HEIGHT_COLUMN = 4;
SI_WEIGHT_COLUMN = 5;
SI_1st_DATA_ROW = 2;

%% Load the data
% Initial progress
fprintf("Forceplate %02d/%02d.", 0, length(forceplate_filepaths));
% Read every file
for numFile = 1 : length(forceplate_filepaths)
    
    % Print progress
    fprintf("\b\b\b\b\b\b%02d/%02d.", numFile, length(forceplate_filepaths));
    
    % Read the whole CSV of iFile-th file
    cellData = readcell(forceplate_filepaths(numFile));
    
    % Reset Forceplate structure
    Forceplate = struct();
    
    % Extract data
    Forceplate.Forces = cell2mat(cellData(FP_1st_DATA_ROW:end, FP_FORCE_COLS));
    Forceplate.Moments = cell2mat(cellData(FP_1st_DATA_ROW:end, FP_MOMENT_COLS));
    Forceplate.COP = cell2mat(cellData(FP_1st_DATA_ROW:end, FP_COP_COLS));
    
    % Extract subject
    subj = "";
    for ss = 1 : NO_SUBJECTS
        if contains(forceplate_filepaths(numFile), sprintf("S%d", ss), 'IgnoreCase', true)
           subj = sprintf("S%d", ss);
        end
    end
    
    % Determine type and store accordingly
    if contains(forceplate_filepaths(numFile), "Calibration", 'IgnoreCase', true)
        Calibration.(subj).raw_forceplate_filename = forceplate_filepaths(numFile);
        Calibration.(subj).Forceplate = Forceplate;
    end
    if contains(forceplate_filepaths(numFile), "Lifting", 'IgnoreCase', true) && ...
       ~contains(forceplate_filepaths(numFile), "SquatLifting", 'IgnoreCase', true)
        Lifting.(subj).raw_forceplate_filename = forceplate_filepaths(numFile);
        Lifting.(subj).Forceplate = Forceplate;
    end
    
    if contains(forceplate_filepaths(numFile), "SquatLifting", 'IgnoreCase', true)
        SquatLifting.(subj).raw_forceplate_filename = forceplate_filepaths(numFile);
        SquatLifting.(subj).Forceplate = Forceplate;
    end
end
fprintf("\n");

% Read every file
% Initial progress
fprintf("Markers %02d/%02d.", 0, length(markers_filepaths));
for numFile = 1 : length(markers_filepaths)
    
    % Print progress
    fprintf("\b\b\b\b\b\b%02d/%02d.", numFile, length(markers_filepaths));
    
    % Read the whole CSV of iFile-th file
    cellData = readcell(markers_filepaths(numFile));
    
    % Get the marker name row
    markerNameRow = cellData(MK_NAMES_ROW, :);
    
    % Reset markers structure
    Markers = struct();
    
    % Go over the columns
    for mkCol = 1 : length(markerNameRow)
        % Skip empty and unlabeled columns
        if ismissing(string(markerNameRow{mkCol})) || isempty(markerNameRow{mkCol}) || contains(markerNameRow{mkCol}, 'Unlabeled', 'IgnoreCase', true)
            continue
        end
        
        % For non-empty columns
        % Separate the OptiTrack Motive MarkerSet name from the Marker name
        semicolIndex = strfind(markerNameRow{mkCol}, MK_MKSET_SEMICOL);
        MSName = markerNameRow{mkCol}(1:semicolIndex-1);
        MName = markerNameRow{mkCol}(semicolIndex+1:end);
        
        % Choose storage index depending on which coordinate this column 
        % corresponds to
        storageIndex = 0;
        if contains(cellData{MK_COORDINATE_ROW, mkCol}, 'X', 'IgnoreCase', true)
            storageIndex = 1;
        elseif contains(cellData{MK_COORDINATE_ROW, mkCol}, 'Y', 'IgnoreCase', true)
            storageIndex = 2;
        elseif contains(cellData{MK_COORDINATE_ROW, mkCol}, 'Z', 'IgnoreCase', true)
            storageIndex = 3;
        end
        
        % Get the cell array of numeric data
        numericData = cellData(MK_1st_DATA_ROW:end, mkCol);
        
        % Replace missing values with nans
        for ii = 1 : length(numericData)
            if ismissing(numericData{ii})
                numericData{ii} = NaN;
            end
        end
        
        % Store the numeric data
        Markers.(MSName).(MName)(:, storageIndex) = cell2mat(numericData);
    end
    
    % Extract subject
    subj = "";
    for ss = 1 : NO_SUBJECTS
        if contains(forceplate_filepaths(numFile), sprintf("S%d", ss), 'IgnoreCase', true)
           subj = sprintf("S%d", ss);
        end
    end
    
    % Determine type and store accordingly
    if contains(markers_filepaths(numFile), "Calibration", 'IgnoreCase', true)
        Calibration.(subj).raw_markers_filename = markers_filepaths(numFile);
        Calibration.(subj).Markers = Markers;
    end
    if contains(markers_filepaths(numFile), "Lifting", 'IgnoreCase', true) && ...
       ~contains(markers_filepaths(numFile), "SquatLifting", 'IgnoreCase', true)
        Lifting.(subj).raw_markers_filename = markers_filepaths(numFile);
        Lifting.(subj).Markers = Markers;
    end
    if contains(markers_filepaths(numFile), "SquatLifting", 'IgnoreCase', true)
        SquatLifting.(subj).raw_markers_filename = markers_filepaths(numFile);
        SquatLifting.(subj).Markers = Markers;
    end
end
fprintf("\n");


% Read every file
% Initial progress
fprintf("Segmentation %02d/%02d.", 0, length(segmentation_filepaths));
for numFile = 3 : length(segmentation_filepaths)
    
    % Print progress
    fprintf("\b\b\b\b\b\b%02d/%02d.", numFile, length(segmentation_filepaths));
    
    % Read the whole CSV of iFile-th file
    cellData = readcell(segmentation_filepaths(numFile));
    
    % Extract subject
    subj = "";
    for ss = 1 : NO_SUBJECTS
        if contains(segmentation_filepaths(numFile), sprintf("S%d", ss), 'IgnoreCase', true)
           subj = sprintf("S%d", ss);
        end
    end
    
    % Determine type and store accordingly
    if contains(segmentation_filepaths(numFile), "Lifting", 'IgnoreCase', true) && ...
       ~contains(segmentation_filepaths(numFile), "SquatLifting", 'IgnoreCase', true)
        Lifting.(subj).raw_segmentation_filename = segmentation_filepaths(numFile);
        Lifting.(subj).Segmentation.TimeLabels = SG_TIME_LABELS;
        % For each rep of set 1
        for numRep = 1 : length(SG_LIFTING_SET1_ROWS)            
            % Get the rep name
            rep = sprintf("Rep%d", numRep);
            % Store data
            Lifting.(subj).Segmentation.Set1.(rep) = cell2mat(cellData(SG_LIFTING_SET1_ROWS(numRep), SG_TIME_COLS));
        end
        % For each rep of set 2
        for numRep = 1 : length(SG_LIFTING_SET2_ROWS)            
            % Get the rep name
            rep = sprintf("Rep%d", numRep);
            % Store data
            Lifting.(subj).Segmentation.Set2.(rep) = cell2mat(cellData(SG_LIFTING_SET2_ROWS(numRep), SG_TIME_COLS));
        end
        % For each rep of set 3
        for numRep = 1 : length(SG_LIFTING_SET3_ROWS)            
            % Get the rep name
            rep = sprintf("Rep%d", numRep);
            % Store data
            Lifting.(subj).Segmentation.Set3.(rep) = cell2mat(cellData(SG_LIFTING_SET3_ROWS(numRep), SG_TIME_COLS));
        end
        % For each rep of set 4
        for numRep = 1 : length(SG_LIFTING_SET1_ROWS)            
            % Get the rep name
            rep = sprintf("Rep%d", numRep);
            % Store data
            Lifting.(subj).Segmentation.Set4.(rep) = cell2mat(cellData(SG_LIFTING_SET4_ROWS(numRep), SG_TIME_COLS));
        end
    end
    
    if contains(segmentation_filepaths(numFile), "SquatLifting_", 'IgnoreCase', true)
        SquatLifting.(subj).raw_segmentation_filename = segmentation_filepaths(numFile);
        SquatLifting.(subj).Segmentation.TimeLabels = SG_TIME_LABELS;
        % For each rep of set 1
        for numRep = 1 : length(SG_SQUATLIFTING_SET1_ROWS)            
            % Get the rep name
            rep = sprintf("Rep%d", numRep);
            % Store data
            SquatLifting.(subj).Segmentation.Set1.(rep) = cell2mat(cellData(SG_SQUATLIFTING_SET1_ROWS(numRep), SG_TIME_COLS));
        end
    end
end
fprintf("\n");

%% Fill the gaps in markers and undersample oversampled forceplate data
%This part of the script:
%   - Memorize sampling rate in the structures
%   - Fills gaps in Markers using the FillCubic function
%   - Rotates the Marker coordinates so that the X-axis is pointing in the
%   posterior-anterior axis, while the Z-axis is pointing along the
%   medio-lateral axis from left to right.
%   - Undersamples Forceplate data if it has more samples than the Markers
%   (as we've had some data that was recorded with 1000Hz on the Forceplate
%   and 100Hz on the Optitrack)

% Calibration subjects
subjCalib = fieldnames(Calibration);
% Lifting subjects
subjLift = fieldnames(Lifting);
% SquatLifting subjects
subjSquatLift = fieldnames(SquatLifting);

% Check each calibration trial
for numSubj = 1 : length(subjCalib)
    
    % Fill the gaps in the Markers
    % Get markerset names
    markerSetNames = fieldnames(Calibration.(subjCalib{numSubj}).Markers);
    
    % For each markerset
    for numMarkerSet = 1 : length(markerSetNames)
        
        % Get marker names
        markerNames = fieldnames(Calibration.(subjCalib{numSubj}).Markers.(markerSetNames{numMarkerSet}));
        
        % For each marker
        for numMarker = 1 : length(markerNames)
            % Fill the gaps
            Calibration.(subjCalib{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}) = ...
            FillGapCubic(Calibration.(subjCalib{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}));
            % Get number of sampels
            numberMarkerSamples = size(Calibration.(subjCalib{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}), 1);
        end
    end
    
    % Undersample forceplate if it was oversampled  
    % Get Forceplate field names
    forceNames = fieldnames(Calibration.(subjCalib{numSubj}).Forceplate);

    % For each field
    for numForce = 1 : length(forceNames)

        % If there are more samples in Forceplate than in Markers
        if size(Calibration.(subjCalib{numSubj}).Forceplate.(forceNames{numForce}), 1) > numberMarkerSamples

            % Resample the Forceplate data with following step
            resize_step = size(Calibration.(subjCalib{numSubj}).Forceplate.(forceNames{numForce}), 1) / numberMarkerSamples;

            % Resample the Forceplate data
            Calibration.(subjCalib{numSubj}).Forceplate.(forceNames{numForce}) = ...
            Calibration.(subjCalib{numSubj}).Forceplate.(forceNames{numForce})(1:resize_step:end, :);
        end
    end
end


% Check each lifting trial
for numSubj = 1 : length(subjLift)
        
    % Fill the gaps in the Markers
    % Get markerset names
    markerSetNames = fieldnames(Lifting.(subjLift{numSubj}).Markers);
    
    % For each markerset
    for numMarkerSet = 1 : length(markerSetNames)
        
        % Get marker names
        markerNames = fieldnames(Lifting.(subjLift{numSubj}).Markers.(markerSetNames{numMarkerSet}));
        
        % For each marker
        for numMarker = 1 : length(markerNames)
            % Fill the gaps
            Lifting.(subjLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}) = ...
            FillGapCubic(Lifting.(subjLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}));
            % Get number of sampels
            numberMarkerSamples = size(Lifting.(subjLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}), 1);
        end
    end
    
    % Undersample forceplate if it was oversampled  
    % Get Forceplate field names
    forceNames = fieldnames(Lifting.(subjLift{numSubj}).Forceplate);

    % For each field
    for numForce = 1 : length(forceNames)

        % If there are more samples in Forceplate than in Markers
        if size(Lifting.(subjLift{numSubj}).Forceplate.(forceNames{numForce}), 1) > numberMarkerSamples

            % Resample the Forceplate data with following step
            resize_step = size(Lifting.(subjLift{numSubj}).Forceplate.(forceNames{numForce}), 1) / numberMarkerSamples;

            % Resample the Forceplate data
            Lifting.(subjCalib{numSubj}).Forceplate.(forceNames{numForce}) = ...
            Lifting.(subjCalib{numSubj}).Forceplate.(forceNames{numForce})(1:resize_step:end, :);
        end
    end
end

% Check each squat lifting trial
for numSubj = 1 : length(subjSquatLift)
        
    % Fill the gaps in the Markers
    % Get markerset names
    markerSetNames = fieldnames(SquatLifting.(subjSquatLift{numSubj}).Markers);
    
    % For each markerset
    for numMarkerSet = 1 : length(markerSetNames)
        
        % Get marker names
        markerNames = fieldnames(SquatLifting.(subjSquatLift{numSubj}).Markers.(markerSetNames{numMarkerSet}));
        
        % For each marker
        for numMarker = 1 : length(markerNames)
            % Fill the gaps
            SquatLifting.(subjSquatLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}) = ...
            FillGapCubic(SquatLifting.(subjSquatLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}));
            % Get number of sampels
            numberMarkerSamples = size(SquatLifting.(subjSquatLift{numSubj}).Markers.(markerSetNames{numMarkerSet}).(markerNames{numMarker}), 1);
        end
    end
    
    % Undersample forceplate if it was oversampled  
    % Get Forceplate field names
    forceNames = fieldnames(SquatLifting.(subjSquatLift{numSubj}).Forceplate);

    % For each field
    for numForce = 1 : length(forceNames)

        % If there are more samples in Forceplate than in Markers
        if size(SquatLifting.(subjSquatLift{numSubj}).Forceplate.(forceNames{numForce}), 1) > numberMarkerSamples

            % Resample the Forceplate data with following step
            resize_step = size(SquatLifting.(subjSquatLift{numSubj}).Forceplate.(forceNames{numForce}), 1) / numberMarkerSamples;

            % Resample the Forceplate data
            SquatLifting.(subjSquatLift{numSubj}).Forceplate.(forceNames{numForce}) = ...
            SquatLifting.(subjSquatLift{numSubj}).Forceplate.(forceNames{numForce})(1:resize_step:end, :);
        end
    end
end

%% Load general info and store it in all structures

% Load the subject info
subjectInfoCellData = readcell(subject_info_filepath);

% Create structure
for numSubj = 1 : NO_SUBJECTS
    % Subject
    subj = sprintf("S%d", numSubj);
    % Load age
    AGE = subjectInfoCellData{SI_1st_DATA_ROW + numSubj - 1, SI_AGE_COLUMN};
    % Load height
    HEIGHT = SI_CM_TO_M(subjectInfoCellData{SI_1st_DATA_ROW + numSubj - 1, SI_HEIGHT_COLUMN});
    % Load weight
    WEIGHT = subjectInfoCellData{SI_1st_DATA_ROW + numSubj - 1, SI_WEIGHT_COLUMN};
    
    Calibration.(subj).AGE = AGE;
    Calibration.(subj).HEIGHT = HEIGHT;
    Calibration.(subj).WEIGHT = WEIGHT;
    
    Lifting.(subj).AGE = AGE;
    Lifting.(subj).HEIGHT = HEIGHT;
    Lifting.(subj).WEIGHT = WEIGHT;
    
    SquatLifting.(subj).AGE = AGE;
    SquatLifting.(subj).HEIGHT = HEIGHT;
    SquatLifting.(subj).WEIGHT = WEIGHT;
    
    % Retain the sampling rate and number of samples
    Calibration.(subj).NumberSamples = size(Calibration.(subj).Forceplate.Forces, 1);
    Calibration.(subj).SamplingTime = SamplingTime;
    Calibration.(subj).SamplingFrequency = SamplingFrequency;
    % Retain the sampling rate
    Lifting.(subj).NumberSamples = size(Lifting.(subj).Forceplate.Forces, 1);
    Lifting.(subj).SamplingTime = SamplingTime;
    Lifting.(subj).SamplingFrequency = SamplingFrequency;
    % Retain the sampling rate
    SquatLifting.(subj).NumberSamples = size(SquatLifting.(subj).Forceplate.Forces, 1);
    SquatLifting.(subj).SamplingTime = SamplingTime;
    SquatLifting.(subj).SamplingFrequency = SamplingFrequency;
end


%% Save the data to output files
%This part of the script:
%   - Saves the Calibration, Lifting and SquatLifting structures inside
%   their own .mat files within the processed_data directory.

% Define save filepaths
calibration_filepath = "../processed_data/Calibration/calibration.mat";
lifting_filepath = "../processed_data/Lifting/lifting.mat";
squatlifting_filepath = "../processed_data/SquatLifting/squatlifting.mat";

% Save the data
fprintf("Saving data...\n");
save(calibration_filepath, "Calibration");
save(lifting_filepath, "Lifting");
save(squatlifting_filepath, "SquatLifting");
fprintf("Saved data.\n");