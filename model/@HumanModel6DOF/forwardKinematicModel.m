function PTS = forwardKinematicModel(obj, q)
%FORWARDKINEMATICMODEL implements the forward kinematic equations for
%the KinematicPointsOfInterest6DOF of the model.
%
%Uses vectorized joint angles angles. Returns a cell array whose elements
%are matrices corresponding to sequences of position vectors of the different 
%points along the trajectory.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   PTS = FORWARDKINEMATICMODEL(q) takes in the matrix of joint coordinates 
%   q (Number of Joints x Number of samples).
%   Returns PTS (1xNpts) a cell array whose elements PTS{i} (2 x Number of
%   samples) contain 2D positions of the KinematicPointsOfInterest6DOF.

    % Extract information about the number of links of the model.
    nLinks = obj.nLinks;

    % Extract information about the trajectory
    [nJoints, nSamples]  = size(q);

    % Number of links should be equal to the number of joints
    if nLinks ~= nJoints
        error("Number of links should be equal to the number of joints");
    end

    % Extract info about points of interest
    nKPOI = length(obj.KPOI);

    % If no points of interest just return
    if nKPOI == 0 
        PTS = {};
        return
    % Otherwise
    % Prealocate output cell array of point forward kinematics (each element is a 3xnSamples matrix)
    else    
        PTS = cell(1, nKPOI);
    end
    
    %%% Extract arrays from KPOI
    % Get an array of which links are the KPOI rigidly linked to
    rigidlyLinkedTo = [obj.KPOI.RigidlyLinkedTo];
    % Get a logical array of whether KPOI relative position are expressed in
    % global frame
    expressedInGlobalFrame = ([obj.KPOI.PositionExpressedInFrame] == 0);
    % Get a matrix of column vectors expressing KPOI positions
    poiPositions = [obj.KPOI.p];
    
    

    %%% Compute Forward Kinematics
    % For the base
    link = 0; 
    ones0 = ones(1, nSamples);
    Plink = find((rigidlyLinkedTo == link));
    Nlink = numel(Plink);

    % For each point belonging to the zeroth link
    for kk = 1 : Nlink
        % Get X and Y coordinates of the current point
        Xc = poiPositions(1, Plink(kk));    % X coordinate of the points who're children to link 0 (in global frame)
        Yc = poiPositions(2, Plink(kk));    % Y coordinate of the points who're children to link 0 (in global frame)

        % Get the global X and Y coordinates of the current point
        PTS{Plink(kk)} = [(Xc .* ones0); (Yc .* ones0); zeros(1, nSamples)];
    end

    % For the first joint
    link = 1;
    sum_q = q(link, :);
    cq = cos(sum_q);
    sq = sin(sum_q);

    % Get the points belonging to 1st link and the number of them
    Plink = find((rigidlyLinkedTo == link));
    Nlink = numel(Plink);

    % For each point belonging to the first link
    for kk = 1 : Nlink
        % Get X and Y coordinates of the current point
        Xc = poiPositions(1, Plink(kk));    % X coordinate of the points who're children to link 1 (along the link)
        Yc = poiPositions(2, Plink(kk));    % Y coordinate of the points who're children to link 1 (perpendicular to the link)

        % Determine if this link is expressed in global
        ee = expressedInGlobalFrame(Plink(kk));

        % When the Global flag is true: Just add the coordinates of the
        % point in the global frame with the relative position vector
        if ee
            PTS{Plink(kk)} = [(Xc); (Yc); zeros(1, nSamples)];
        else
            % When the global flag is false:
            % Calculate the global X and Y coordinates of the current point
            %     PTS{Pjj(kk)}(1, :) = (Xc .* cq - Yc .* sq);
            %     PTS{Pjj(kk)}(2, :) = (Xc .* sq + Yc .* cq);
            %     PTS{Pjj(kk)}(3, :) = zeros(1, nSamples);
            PTS{Plink(kk)} = [(Xc .* cq - Yc .* sq); (Xc .* sq + Yc .* cq); zeros(1, nSamples)];
        end


        PTS{Plink(kk)} = [(Xc .* cq - Yc .* sq); (Xc .* sq + Yc .* cq); zeros(1, nSamples)];
    end

    % Calculate the global X and Y coordinates of the 1st distal link
    % end, otherwise known as the center of the 2nd joint
    Gx = obj.L(link) .* cq;
    Gy = obj.L(link) .* sq;

    % For subsequent joints, you must also take into account the position
    % of the link referential frame
    for link = 2 : nJoints+1        
        % Sum the joint angles from 1 to jj (or when jj > nJoints, sum to nJoints, because there is the end effector link)    
        sum_q = sum(q(1:min(link, nJoints), :), 1);
        cq = cos(sum_q);
        sq = sin(sum_q);

        % Get the points belonging to jjth link and the number of them
        Plink = find((rigidlyLinkedTo == link));
        Nlink = numel(Plink);

        % For each point belonging to the first link
        for kk = 1 : Nlink
            % Get X and Y coordinates of the current point
            Xc = poiPositions(1, Plink(kk));    % X coordinate of the points who're children to link jj (along the link)
            Yc = poiPositions(2, Plink(kk));    % Y coordinate of the points who're children to link jj (perpendicular to the link)

            % Determine if this link is expressed in global
            ee = expressedInGlobalFrame(Plink(kk));

            % When the Global flag is true: Just add the coordinates of the
            % point in the global frame with the relative position vector
            if ee
                PTS{Plink(kk)} = [(Gx + Xc); (Gy + Yc); zeros(1, nSamples)];
            else
                % When the global flag is false:
                % Calculate the global X and Y coordinates of the current point by
                % adding its position relative to its parent link frame to the
                % global position of the parent link frame
                PTS{Plink(kk)} = [(Gx + Xc .* cq - Yc .* sq); (Gy + Xc .* sq + Yc .* cq); zeros(1, nSamples)];
            end
        end

        % Update the global position of the local link frame to the
        % distal end of the jjth link, otherwise known as the center of
        % the (jj+1)th joint
        % EDIT: Only when jj is less than nJoints
        if link <= nJoints
            Gx = Gx + obj.L(link) .* cq;
            Gy = Gy + obj.L(link) .* sq;
        end
    end

end