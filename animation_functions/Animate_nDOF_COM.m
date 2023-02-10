function Animate_nDOF_COM(q,L,M,CMP,Ts,varargin)
%ANIMATE_NDOF_COM animates a n-DOF planar robot as well as its segment
%centers of mass and overall center of mass, given the joint position
%vectors at successive samples, the masses of the segments, the positions 
%of the centers of mass in the segment reference frames, as well as the 
%robot segment lengths and the sampling rate.
%
%   ANIMATE_NDOF_COM(q,L,M,CMP,Ts) takes in the matrix of joint angles q, 
%   the size being (n x Number of samples), the nD vector of segment 
%   lengths L, the nD vector of segment masses M, the 3xn matrix of center 
%   of mass positions in the segment reference frames, and sampling rate Ts
%   in seconds (should be superior to 0.001s).
%   
%   ANIMATE_NDOF_COM(q,L,M,CMP,Ts,options) allows the possibility of 
%   providing optional arguments such as:
%       - body_thickness, body_color, joint_size, joint_thickness, 
%       joint_color: correspond to LineWidth and Color of body, MarkerSize,
%       LineWidth and Color of joints respectively
%       - legend_entry, xlabel, ylabel, title : corresponding plot elements
%       - show_legend, show_grid : to show or not the corresponding plot
%       elements
%       - save_path : where the function should save the animation (without
%       file extension)
%       - show_body_com : flag whether to plot global center of mass
%       - com_marker, com_color, com_legend_entries, com_size, 
%       com_thickness: marker shape, the colormap to use (with number of 
%       com elements to draw), the MarkerSize, the LineWidth.
%       - com_segment_labels : labels for the COMs

% #Input check and initialization of the options structure: options
if nargin > 5
    % Initialize the options variable with the value of the additional argument
    options = varargin{1};
else
    % Create empty options
    options = [];
end

% Check inputs and put default values if not given
if isfield(options, 'show_body_com'); show_body_com = options.show_body_com; else; show_body_com = true; end
if isfield(options, 'com_marker'); com_marker = options.com_marker; else; com_marker = 'o'; end
if isfield(options, 'com_size'); com_size = options.com_size; else; com_size = 10; end
if isfield(options, 'com_thickness'); com_thickness = options.com_thickness; else; com_thickness = 1; end
if isfield(options, 'com_segment_labels'); com_segment_labels = options.com_segment_labels; else; com_segment_labels = arrayfun(@(a) ['S', num2str(a)], 1:size(q, 1), 'UniformOutput', false); end

% Number of markers needed to represent the COMs will be
NumMark = size(q, 1);

% Structure for storing COM values
COM_Markers = [];

% Get centers of mass
if show_body_com
    % Get both segment COMs and body COM
    [COMs, COM] = COMs_nDOF_Tensor(q, L, CMP, M);
    % Store body COM
    COM_Markers.BODY.COM = COM.';
    
    % Number of markers will be 1 more
    NumMark = NumMark + 1;
else
    % Get only segment COMs
    COMs = COMs_nDOF_Tensor(q, L, CMP);
end

% Store body COMs
for ii = 1 : size(q, 1)
    COM_Markers.COM.(com_segment_labels{ii}) = squeeze(COMs(:, :, ii)).';
end

% Additional input check
if isfield(options, 'com_color'); com_color = options.com_color; else; com_color = colorcube(NumMark+1); end


% Modify options to be able to transmit to markers function
options.markers_marker = com_marker;
options.markers_color = com_color;
options.markers_separate_legend = true;
options.markers_size = com_size;
options.markers_thickness = com_thickness;

% Animate
Animate_nDOF_Markers(q,COM_Markers,L,Ts,options);
end

