function D = collisionSpheresDistances(obj, q)
%COLLISIONSPHERESDISTANCES calculates the distances between collision 
%spheres for the HumanModel6DOF, given the joint angles along the
%trajectory.
%
%   D = COLLISIONSPHERESDISTANCES(obj, q) takes in the vectorized joint
%   trajectories (6xNbSamples).
%   Returns a cell matrix D whose elements are arrays containing distances
%   between spheres across time, with the indices of the cell matrix
%   corresponding to the spheres whose distance is being calculated.

% Get the collision spheres forward kinematics
CTR = obj.collisionForwardKinematicModel(q);

% Exctract useful constants
Ns = size(obj.CS, 2);      % Num. spheres

% Prealocate output tensor of distances across time
D = cell(Ns, Ns);  % Prealocate output (all elements 1xN)

% For all spheres calculate distances to spheres that were not checked yet
for ss1 = 1 : Ns - 1
    % Get current sphere 1 trajectory
    C1 = CTR{ss1};
    
    % Spheres that have not yet been compared to current sphere 1
    for ss2 = ss1 + 1 : Ns
        % Get current sphere 2 trajectory
        C2 = CTR{ss2};
        
        % Calculate the distance as the distance between the centers minus
        % the sum of the radii
        D{ss1, ss2} = sqrt(sum((C1-C2).^2)) - (obj.CS(ss1).radius + obj.CS(ss2).radius);        
    end
end
end

