function obj = setInertialParametersFromAnthropometricTables(obj, weight, height)
%SETINERTIALPARAMETERSFROMANTHROPOMETRICTABLES loads the parameters of the 
%human model using an anthropometric table from Dumas (2018).

    %Extract geometrical parameters using the table from Dumas, 2018 for
    %male subjects, along with the subject height and weight. The table 
    %parameters are stored in a file called Parametres.xlsx.
    nargin
    %% Treat input weight and height
    if nargin == 3
        obj.WEIGHT = weight;
        obj.HEIGHT = height;
    end    
    WEIGHT = obj.WEIGHT;
    HEIGHT = obj.HEIGHT;

    
    %% Mean subject height from Dumas 2007, 2018
    msh = 1.77;

    %% Get the data from the csv file
    [numericData, fullTextData, ~] = xlsread('/AnthropometricTable.xlsx');

    % Get all text except the column and row names
    textData = fullTextData(2:end, 2:end);

    % Convert imaginary numbers from text to double
    for ii = 1 : size(numericData, 1)
        for jj = 1 : size(numericData, 2)

            % If a NaN is in numericData, convert the corresponding textData
            % from text to (complex) double
            if isnan(numericData(ii, jj))
                numericData(ii,jj) = str2double(textData{ii, jj});
            end
        end
    end

    % Create structures for all the body segments (all rows of the table)
    for ii = 1 : size(fullTextData, 1)-1

        % If current row name contains spaces replace with underscore
        fullTextData{ii+1, 1} = strrep(fullTextData{ii+1, 1}, ' ', '_');

        % SEGMENT LENGTHS IN [m]
        % Create segment lengths (height * mean seg len / mean subj height)
        L.(fullTextData{ii+1, 1}) = numericData(ii, 1) * (HEIGHT/msh);

        % SEGMENT MASSES IN [kg]
        % Create segment masses (weight * segm mass percentage / 100)
        M.(fullTextData{ii+1, 1}) = WEIGHT*(numericData(ii, 2)/100);

        % COM POSITIONS IN [% Segment Length]
        % Create segment centers of mass in percentages (perc of seg length / 100)
        COM.(fullTextData{ii+1, 1}) = (numericData(ii, 3:5)/100).'; % Column vector

        % Create segment intertia matrices: 
        % GYRATION MATRIX IN [% Segment Length]
        % a) Gyration matrix
        G = diag((numericData(ii, 6:8) / 100).^2) / 2;  % Divide by because of the symmetric addition a few lines down
        G(1, 2) = (numericData(ii, 9) / 100)^2;
        G(1, 3) = (numericData(ii, 10) / 100)^2;
        G(2, 3) = (numericData(ii, 11) / 100)^2;
        G = G + G.';    % Make symmetric by addition

        % INERTIA MATRIX IN [m^2.kg]
        % b) Inertia Matrix
        % (seg mass * seg len^2 * gyration matrix in percentages^2 / 100^2)
        IM.(fullTextData{ii+1, 1}) = M.(fullTextData{ii+1, 1}) * L.(fullTextData{ii+1, 1})^2 * G;
    end

    % Change the reference frames of the COM and IM for certain segments. The
    % segment reference frames are given within Dumas, 2018, and the calculated
    % IM are given w.r.t. to the segment reference frame orientation, at the
    % COM position.
    % We need a Modified Denavit-Hartenberg type orientation, so all the segment 
    % reference frames need to be oriented such that the X axis goes along the
    % segment. All segment reference frames must be placed at the proximal end 
    % of the segment.

    % Segments for which segment reference frame should be moved from distal to
    % proximal end
    dtp_segments = {'Shank', 'Thigh', 'Pelvis', 'Abdomen', 'Thorax'};

    % Arm segments, for which the rotation of the segment reference frames needs
    % to be changed differently than for other segments
    arm_segments = {'Upper_Arm', 'Forearm', 'Hand'};

    % Rotate by -90 degrees around the Z axis
    Rz_minus90 = rotz(-90);
    % Rotate by +90 degrees around the Z axis
    Rz_plus90 = rotz(90);

    % Segment names
    segments = fieldnames(COM);

    %%%%%  WARNING
    % GOOD: SEGMENT LENGTHS IN [m]
    % GOOD: SEGMENT MASSES IN [kg]
    % BAD:  COM POSITIONS IN [% Segment Length]
    % GOOD: INERTIA MATRIX IN [m^2.kg]

    % For all segments
    for ii = 1 : length(segments)

        % If segment belongs to the set of segments for which we need to
        % change the segment frame position from distal to proximal end
        if ismember(segments{ii}, dtp_segments)
            % Modify the CoM Y-coordinate (expressed in percentages), by adding
            % 1 to it (since we are moving from distal to proximal end, we are
            % effectively moving along a vector in the direction of -y for a
            % distance of the segment length, which in percentages is 1).
            % The X and Z coordinates remain unchanged.
           COM.(segments{ii})(2) = 1 + COM.(segments{ii})(2);
        end

        % If segment belongs to the set of arm segments for which a rotation
        % matrix of +90 degrees is needed
        if ismember(segments{ii}, arm_segments)
            % Adjust orientation of frame in which CoM is expressed
            COM.(segments{ii}) = Rz_plus90 * COM.(segments{ii});

            % Adjust orientation of frame in which the IM is expressed (formula
            % number 5 in Dumas 2018)
            IM.(segments{ii}) = Rz_plus90 * IM.(segments{ii}) * Rz_plus90.';
        % Otherwise use the -90 degrees rotation matrix
        else
            % Adjust orientation of frame in which CoM is expressed
            COM.(segments{ii}) = Rz_minus90 * COM.(segments{ii});

            % Adjust orientation of frame in which the IM is expressed (formula
            % number 5 in Dumas 2018)
            IM.(segments{ii}) = Rz_minus90 * IM.(segments{ii}) * Rz_minus90.';
        end
        
        % For all segments
        % Adjust the position of frame in which the IM is expressed (formula
        % number 6 in Dumas 2018)
        IM.(segments{ii}) = IM.(segments{ii}) + M.(segments{ii}) * ...
        (( (COM.(segments{ii}) * L.(segments{ii})).' * (COM.(segments{ii}) * L.(segments{ii}))) * eye(3) - ...
        (COM.(segments{ii}) * L.(segments{ii})) * (COM.(segments{ii}) * L.(segments{ii})).');
    end
    
    %%%%%  WARNING
    % COM POSITIONS IN [% Segment Length]
    
    % For all segments transpose the vectors
    for ii = 1 : length(segments)
        COM.(segments{ii}) = COM.(segments{ii}).';
    end
    
    % For all segments multiply by segment length
    for ii = 1 : length(segments)
        COM.(segments{ii}) = COM.(segments{ii}).' * L.(segments{ii});
    end


    % Merge abdomen and pelvis
    % Length and mass
    L.("Pelvis_and_Abdomen") = L.("Pelvis") + L.("Abdomen");
    M.("Pelvis_and_Abdomen") = M.("Pelvis") + M.("Abdomen");
    % Displacement of frames
    r.("Abdomen_to_Pelvis") = [L.("Pelvis"); 0; 0]; % The abdomen frame is along the x axis of the pelvis frame
    % Center of mass
    COM.("Abdomen_in_Pelvis_frame") = COM.("Abdomen") + r.("Abdomen_to_Pelvis");   
    COM.("Pelvis_and_Abdomen") = 1 / (M.("Pelvis") + M.("Abdomen")) .* (M.("Pelvis") * COM.("Pelvis") + ...
                                  M.("Abdomen") * COM.("Abdomen_in_Pelvis_frame")); % Take weighted average of the COMs
    % Intertia matrix
    IM.("Abdomen_in_Pelvis") = IM.("Abdomen") + M.("Abdomen") .* ...
                               (r.("Abdomen_to_Pelvis").' * r.("Abdomen_to_Pelvis") * eye(3) - ...
                                r.("Abdomen_to_Pelvis") * r.("Abdomen_to_Pelvis").');
    IM.("Pelvis_and_Abdomen") = IM.("Pelvis") + IM.("Abdomen");
    
    %% Perform checks:
    % 1. X component of COM >=0 (for all except Foot)
    % 2. Are all IM symmetric and positive definite
    for ii = 1 : length(segments)
        if ~isequal(segments{ii}, 'Foot')
            if ~(COM.(segments{ii})(1) > 0)
                error("Segment %s CoM isn't along the x-axis.", segments{ii});
            end
            if norm(IM.(segments{ii}) - IM.(segments{ii}).', 'fro') > 1e-6
                error("Segment %s inertia matrix isn't symmetric (up to 1e-6 in Frobenius norm).", segments{ii});
            end
            if ~all(eig(IM.(segments{ii}))>0)
                error("Segment %s inertia matrix isn't positive semidefinite.", segments{ii});
            end
        end
    end

    % Set segment lengths
    obj.LFOOT = L.("Foot");
    obj.L = [L.("Shank"); 
             L.("Thigh"); 
             L.("Pelvis_and_Abdomen"); 
             L.("Thorax"); 
             L.("Upper_Arm"); 
             L.("Forearm")];
    obj.LHAND = L.("Hand");
    obj.LHEAD = L.("Head_with_Neck");

    % Set segment masses (double masses for bilateral segments)
    obj.MFOOT = 2 * M.("Foot");
    obj.M = [2 * M.("Shank"); 
             2 * M.("Thigh"); 
             M.("Pelvis_and_Abdomen"); 
             M.("Thorax"); 
             2 * M.("Upper_Arm"); 
             2 * M.("Forearm")];
    obj.MHAND = 2 * M.("Hand");
    obj.MHEAD = M.("Head_with_Neck");
    
    % Set segment COMs
    obj.CoMFOOT = reshape(COM.("Foot")(1:2), 2, 1);
    obj.CoM = [reshape(COM.("Shank")(1:2), 2, 1),...
                reshape(COM.("Thigh")(1:2), 2, 1),...
                reshape(COM.("Pelvis_and_Abdomen")(1:2), 2, 1),...
                reshape(COM.("Thorax")(1:2), 2, 1),...
                reshape(COM.("Upper_Arm")(1:2), 2, 1),...
                reshape(COM.("Forearm")(1:2), 2, 1)];
    obj.CoMHAND = reshape(COM.("Hand")(1:2), 2, 1);
    obj.CoMHEAD = reshape(COM.("Head_with_Neck")(1:2), 2, 1);
    
    % Set segment Z-axis inertias (double inertias for bilateral segments)
    obj.IzzFOOT = 2 * IM.("Foot")(3, 3);
    obj.Izz = [2 * IM.("Shank")(3, 3); 
               2 * IM.("Thigh")(3, 3); 
               IM.("Pelvis_and_Abdomen")(3, 3); 
               IM.("Thorax")(3, 3); 
               2 * IM.("Upper_Arm")(3, 3); 
               2 * IM.("Forearm")(3, 3)];
    obj.IzzHAND = 2 * IM.("Hand")(3, 3);
    obj.IzzHEAD = IM.("Head_with_Neck")(3, 3);
end