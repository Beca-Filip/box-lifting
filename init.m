%INIT defines the directory of the imports

% Add CASADI
casadi_dir = "..\casadi-windows-matlabR2016a-v3.5.5";
addpath(casadi_dir);

% Add all paths in this directory
this_dir = '.';
addpath(genpath(this_dir));