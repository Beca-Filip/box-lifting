function [V, J] = forwardKinematicVelocityModel(obj,q,dq)
%FORWARDKINEMATICVELOCITYMODEL implements the forward kinematic velocity 
%equations for KinematicPointsOfInterest6DOF of the model.
%
%Uses vectorized joint angles and velocities. Returns a cell array whose 
%elements are matrices corresponding to sequences of velocity vectors of 
%the different points along the trajectory.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   V = FORWARDKINEMATICVELOCITYMODEL(obj,q,dq) takes in the matrix of 
%   joint coordinates q (Number of Joints x Number of samples) and of joint
%   velocities dq (Number of Joints x Number of samples).
%   Returns V (1xNpts) a cell array whose elements V{i} (2 x Number of
%   samples) contain 2D velocities of the KinematicPointsOfInterest6DOF.

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
        V = {};
        J = {};
        return
    % Otherwise
    % Prealocate output cell array of point forward kinematics (each element is a 3xnSamples matrix)
    else    
        V = cell(1, nKPOI);
        % Prealocate jacobians for each point at all times
        J = cell(nKPOI, nSamples);
    end
    
    %%% Extract arrays from KPOI
    % Get an array of which links are the KPOI rigidly linked to
    rigidlyLinkedTo = [obj.KPOI.RigidlyLinkedTo];
    % Get a matrix of column vectors expressing KPOI positions
    poiPositions = [obj.KPOI.p];
            
    % For all times
    for timeStep = 1 : nSamples
        % For all joints
        for currLink = 1 : nJoints+1
            % Sum the joint angles from 1 to link (or when link > nJoints, sum to nJoints, because there is the end effector segment)    
            sum_q = sum(q(1:min(currLink, nJoints), timeStep), 1);
            cq = cos(sum_q);
            sq = sin(sum_q);

            % Get the points belonging to jjth segment and the number of them
            poiOfCurrLink = find((rigidlyLinkedTo == currLink));
            numPoiOfCurrLink = numel(poiOfCurrLink);

            % Perform the computations only if there are points on this segment
            if numPoiOfCurrLink > 0
                % Create a sum variable
                sum_x = 0;
                sum_y = 0;

                % Calculate backwards the entries of the jacobians
                for currJoint = nJoints : -1 : 1
                    % If the current entry is earlier in the chain than the current joint
                    if currJoint < currLink
                        % Modify the sum of all q's up to currLink
                        sum_q_currLink = sum(q(1:currJoint, timeStep));
                        sq_currLink = sin(sum_q_currLink);
                        cq_currLink = cos(sum_q_currLink);

                        % Add the contribution to the overall velocity of the current entry to the joint
                        sum_x = sum_x - obj.L(currJoint) * sq_currLink;
                        sum_y = sum_y + obj.L(currJoint) * cq_currLink;
                    end
                    
                    % If the current entry is matching the current joint
                    if currJoint == currLink
                        % Add the contribution to the current entry of the points' relative position to the joint
                        sum_x = sum_x - poiPositions(1, poiOfCurrLink).' .* sq - poiPositions(2, poiOfCurrLink).' .* cq;
                        sum_y = sum_y + poiPositions(1, poiOfCurrLink).' .* cq - poiPositions(2, poiOfCurrLink).' .* sq;
                    end
                    
                    % Set the current Jacobian entry at current time for each point
                    for currPoi = 1 : numPoiOfCurrLink
                        if size(sum_x, 1) > 1 && size(sum_y, 1) > 1
                            J{poiOfCurrLink(currPoi), timeStep} = [[sum_x(currPoi); sum_y(currPoi)], J{poiOfCurrLink(currPoi), timeStep}];
                        else
                            J{poiOfCurrLink(currPoi), timeStep} = [[sum_x; sum_y], J{poiOfCurrLink(currPoi), timeStep}];
                        end
                    end
                end
            end
        end
    end
    
    % For each point
    for currPoi = 1 : nKPOI
        % Initialize the current velocity vector to empty array
        V{currPoi} = [];
        % For each time
        for timeStep = 1 : nSamples
            % Concatenate the velocity at time tt to the velocity array
            V{currPoi} = [V{currPoi}, J{currPoi, timeStep} * dq(:, timeStep)];
        end
    end

end