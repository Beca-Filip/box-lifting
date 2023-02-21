function PTS = Points_FKM_nDOF_Tensor(q,L,C,PC)
%POINTS_FKM_NDOF_TENSOR implements the forward kinematic equations for
%given points fixed to various segments of an n-DOF planar manipulator,
%using vectorized joint angles angles. Returns a tensor whose slices are 
%matrices corresponding to sequences of position vectors of the different 
%points along the trajectory.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   PTS = POINTS_FKM_NDOF_TENSOR(q,L,C,PC) takes in the matrix of joint 
%   angles q (Number of Joints x Number of samples) alongside the nD vector of 
%   segment lengths L, a (3 x Number of points) matrix of points' relative 
%   positions with respect to their parent segment C, and a (1 x Number of
%   points) vector containing indices of the parent segments of the points 
%   in C : PC.

    
% Exctract useful constants
n = size(q, 1); % Number of Segments
N = size(q, 2); % Number of Samples
nP = size(C, 2);    % Number of points

% Prealocate output tensor of Point forward kinematics
PTS = zeros(3, N, nP);  % Prealocate output


% Compute positions

% For the 0th joint, meaning the base
jj = 0;
ones0 = ones(1, N);
Pjj = find((PC == jj));
Njj = numel(Pjj);

% For each point belonging to the zeroth segment
for kk = 1 : Njj
    % Get local X and Y coordinates of the current point
    Xc = C(1, Pjj(kk));    % X coordinate of the points who're children to segment 0 (in global frame)
    Yc = C(2, Pjj(kk));    % Y coordinate of the points who're children to segment 0 (in global frame)

    % Get the global X and Y coordinates of the current point
    PTS(1, :, Pjj(kk)) = Xc .* ones0;
    PTS(2, :, Pjj(kk)) = Yc .* ones0;
end

% For the first joint
jj = 1;
sum_q = q(jj, :);
cq = cos(sum_q);
sq = sin(sum_q);

% Get the points belonging to 1st segment and the number of them
Pjj = find((PC == jj));
Njj = numel(Pjj);

% For each point belonging to the first segment
for kk = 1 : Njj
    % Get local X and Y coordinates of the current point
    Xc = C(1, Pjj(kk));    % X coordinate of the points who're children to segment 1 (along the segment)
    Yc = C(2, Pjj(kk));    % Y coordinate of the points who're children to segment 1 (perpendicular to the segment)

    % Calculate the global X and Y coordinates of the current point
    PTS(1, :, Pjj(kk)) = (Xc .* cq - Yc .* sq);
    PTS(2, :, Pjj(kk)) = (Xc .* sq + Yc .* cq);
end

% Calculate the global X and Y coordinates of the 1st distal segment
% end, otherwise known as the center of the 2nd joint
Gx = L(jj) .* cq;
Gy = L(jj) .* sq;

% For subsequent joints, you must also take into account the position
% of the segment referential frame
for jj = 2 : n        
    % Sum the joint angles from 1 to jj       
    sum_q = sum(q(1:jj, :), 1);
    cq = cos(sum_q);
    sq = sin(sum_q);
    
    % Get the points belonging to jjth segment and the number of them
    Pjj = find((PC == jj));
    Njj = numel(Pjj);

    % For each point belonging to the first segment
    for kk = 1 : Njj
        % Get local X and Y coordinates of the current point
        Xc = C(1, Pjj(kk));    % X coordinate of the points who're children to segment jj (along the segment)
        Yc = C(2, Pjj(kk));    % Y coordinate of the points who're children to segment jj (perpendicular to the segment)

        % Calculate the global X and Y coordinates of the current point by
        % adding its position relative to its parent segment frame to the
        % global position of the parent segment frame
        PTS(1, :, Pjj(kk)) = Gx + Xc .* cq - Yc .* sq;
        PTS(2, :, Pjj(kk)) = Gy + Xc .* sq + Yc .* cq;
    end

    % Update the global position of the local segment frame to the
    % distal end of the jjth segment, otherwise known as the center of
    % the (jj+1)th joint
    Gx = Gx + L(jj) .* cq;
    Gy = Gy + L(jj) .* sq;
end

end

