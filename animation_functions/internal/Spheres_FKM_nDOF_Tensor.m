function SPH = Spheres_FKM_nDOF_Tensor(q,L,spheres,Nps)
%SPHERES_FKM_NDOF_TENSOR implements the forward kinematic equations for
%given spheres fixed to various segments of an n-DOF planar manipulator,
%using vectorized joint angles angles. Returns a tensor whose 3D slices are 
%tensors corresponding to sequences of position vectors of the different 
%points of the spheres along the trajectory.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   PTS = SPHERES_FKM_NDOF_TENSOR(q,L,spheres,Nps) takes in the matrix of joint 
%   angles q (Number of Joints x Number of samples) alongside the nD vector of 
%   segment lengths L, a spheres structure, and the number of equally
%   spaced points along the sphere.


%   A spheres structure contains fields:
%       - centers: (3 x Number of spheres) matrix of sphere centers' relative 
%       positions with respect to their parent segment.
%       - radii : (1 x Number of spheres) vector containin the radii of the
%       spheres.
%       - parent_segment: and a (1 x Number of spheres) vector containing 
%       indices of the parent segments of the spheres.
   

% Exctract useful constants
n = size(q, 1); % Number of Segments
N = size(q, 2); % Number of Samples
Ns = size(spheres.centers, 2);    % Number of spheres
Nps = Nps;      % Number of points per sphere

% Get the forward kinematics of the centers
CTR = Points_FKM_nDOF_Tensor(q,L,spheres.centers,spheres.parent_segment);

% Prealocate the forward kinematics of the sphere points
SPH = zeros(3, N, Nps, Ns);

% Prealocate a unit circle with demanded number of points per sphere
t = linspace(0, 2*pi, Nps);
ux = cos(t);
uy = sin(t);
uz = zeros(size(t));

% For each sphere
for ii = 1 : Ns
    
    % Get the adapted circle
    xs = spheres.radii(ii) * ux;
    ys = spheres.radii(ii) * uy;
    zs = uz;
    
    % Get the center's position across time
    C = squeeze(CTR(:, :, ii));
    
    % Add the circle points to the center points at each timestep
    for tt = 1 : N
        SPH(:, tt, :, ii) = C(:, tt) + [xs; ys; zs];
    end
end

end

