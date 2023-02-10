function [A, J, dJ] = forwardKinematicAccelerationModel(obj,q,dq,ddq)
%FORWARDKINEMATICACCELERATIONMODEL implements the forward kinematic 
%acceleration equations for the KinematicPointsOfInterest6DOF of the model.
%
%Uses vectorized joint angles, velocities, and accelerations. Returns a 
%cell array whose elements are matrices corresponding to sequences of 
%acceleration vectors of the different points along the trajectory.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   [A, J, dJ] = POINTS_ACCELERATION_NDOF_CELL(obj,q,dq,ddq) takes in the 
%   matrix of joint angles q (Number of Joints x Number of samples), of 
%   joint velocities dq (Number of Joints x Number of samples), and of
%   joint accelerations ddq (Number of Joints x Number of samples).
%   Returns A (1xNpts) a cell array whose elements A{i} (2 x Number of
%   samples) contain 2D accelerations of the KinematicPointsOfInterest6DOF.

    
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
        A = {};
        J = {};
        dJ = {};
        return
    % Otherwise
    % Prealocate output cell array of point forward kinematics (each element is a 3xnSamples matrix)
    else    
        A = cell(1, nKPOI);
        % Prealocate jacobians for each point at all times
        J = cell(nKPOI, nSamples);
        % Prealocate derivative of jacobian for each point at all times
        dJ = cell(nKPOI, nSamples);
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
            % Sum the joint angles from 1 to jj (or when jj > nJoints, sum to nJoints, because there is the end effector segment)    
            sum_q = sum(q(1:min(currLink, nJoints), timeStep), 1);
            cq = cos(sum_q);
            sq = sin(sum_q);

            % Sum the joint velocities from 1 to jj (or when jj > nJoints, sum to nJoints, because there is the end effector segment)
            sum_dq = sum(dq(1:min(currLink, nJoints), timeStep), 1);

            % Get the points belonging to jjth segment and the number of them
            poiOfCurrLink = find((rigidlyLinkedTo == currLink));
            numPoiOfCurrLink = numel(poiOfCurrLink);

            % Perform the computations only if there are points on this segment
            if numPoiOfCurrLink > 0
                % Create a sum variable for velocities
                sum_v_x = 0;
                sum_v_y = 0;
                % Create a sum variable for accelerations
                sum_a_x = 0;
                sum_a_y = 0;

                % Calculate backwards the entries of the jacobians
                for currJoint = nJoints : -1 : 1
                    % If the current entry is earlier in the chain than the current joint
                    if currJoint < currLink
                        % Modify the sum of all q's up to ll
                        sum_q_currLink = sum(q(1:currJoint, timeStep));
                        sq_currLink = sin(sum_q_currLink);
                        cq_currLink = cos(sum_q_currLink);

                        % Modify the sum of all dq's up to ll
                        sum_dq_currLink = sum(dq(1:currJoint, timeStep));

                        % Add the contribution to the current entry of J, of the joint ll
                        sum_v_x = sum_v_x - obj.L(currJoint) * sq_currLink;
                        sum_v_y = sum_v_y + obj.L(currJoint) * cq_currLink;

                        % Add the contribution to the current entry of dJ, of the joint ll
                        sum_a_x = sum_a_x - obj.L(currJoint).' .* cq_currLink .* sum_dq_currLink;
                        sum_a_y = sum_a_y - obj.L(currJoint).' .* sq_currLink .* sum_dq_currLink;
                    end
                    
                    % If the current entry is matching the current joint
                    if currJoint == currLink
                        % Add the contribution to the current entry of J, of the current points' relative position to the jj-th joint
                        sum_v_x = sum_v_x - poiPositions(1, poiOfCurrLink).' .* sq - poiPositions(2, poiOfCurrLink).' .* cq;
                        sum_v_y = sum_v_y + poiPositions(1, poiOfCurrLink).' .* cq - poiPositions(2, poiOfCurrLink).' .* sq;

                        % Add the contribution to the current entry of dJ, of the current point's relative position to the jj-th joint
                        sum_a_x = sum_a_x - poiPositions(1, poiOfCurrLink).' .* cq .* sum_dq + poiPositions(2, poiOfCurrLink).' .* sq .* sum_dq;
                        sum_a_y = sum_a_y - poiPositions(1, poiOfCurrLink).' .* sq .* sum_dq - poiPositions(2, poiOfCurrLink).' .* cq .* sum_dq;
                    end

                    % Set the current Jacobian entry at current time for each point
                    for currPoi = 1 : numPoiOfCurrLink
                        if size(sum_v_x, 1) > 1 && size(sum_v_y, 1) > 1
                            J{poiOfCurrLink(currPoi), timeStep} = [[sum_v_x(currPoi); sum_v_y(currPoi)], J{poiOfCurrLink(currPoi), timeStep}];
                        else
                            J{poiOfCurrLink(currPoi), timeStep} = [[sum_v_x; sum_v_y], J{poiOfCurrLink(currPoi), timeStep}];
                        end
                    end
                    % Set the current derivative-of-Jacobian entry at current time for each point
                    for currPoi = 1 : numPoiOfCurrLink
                        if size(sum_a_x, 1) > 1 && size(sum_a_y, 1) > 1
                            dJ{poiOfCurrLink(currPoi), timeStep} = [[sum_a_x(currPoi); sum_a_y(currPoi)], dJ{poiOfCurrLink(currPoi), timeStep}];
                        else
                            dJ{poiOfCurrLink(currPoi), timeStep} = [[sum_a_x; sum_a_y], dJ{poiOfCurrLink(currPoi), timeStep}];
                        end
                    end
                end
            end
        end
    end
    
    % For each point
    for currPoi = 1 : nKPOI
        % Initialize the acceleration velocity vector to empty array
        A{currPoi} = [];
        % For each time
        for timeStep = 1 : nSamples
            % Concatenate the velocity at time tt to the velocity array
            A{currPoi} = [A{currPoi}, dJ{currPoi, timeStep} * dq(:, timeStep) + J{currPoi, timeStep} * ddq(:, timeStep)];
        end
    end

end