clear all;
close all;
clc;

% Constructor parameters
nTrials = 1;
nPrimal = 60;
nDual = 3000;

% Create an initial guess
igdoc = InitialGuessDOC(nTrials, nPrimal, nDual);

disp(igdoc)
disp(igdoc.primal.');

function_test_InitialGuessDOC_updatePrimalDual(igdoc);

disp(igdoc)
disp(igdoc.primal.');