clear all;
close all;
clc;

% Load lifting trials
Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

% Initialize trial index
TrialIndex = 1;

% Get segment lengths
R = Trials(TrialIndex).humanModel.R;
p = Trials(TrialIndex).humanModel.p;
WEIGHT = Trials(TrialIndex).humanModel.WEIGHT;
HEIGHT = Trials(TrialIndex).humanModel.HEIGHT;
L = Trials(TrialIndex).humanModel.L;
Trials(TrialIndex).humanModel = HumanModel6DOF(R, p, WEIGHT, HEIGHT);

% Set default collision spheres for the human model of trials
Trials(TrialIndex).humanModel = Trials(TrialIndex).humanModel.setDefaultCollisionSpheres();

% Create box collision sphere
boxCollisionSphere = Trials(TrialIndex).liftingEnvironment.createBoxCollisionSphere();

% Add the collision sphere to the human model
Trials(TrialIndex).humanModel = Trials(TrialIndex).humanModel.addCollisionSphere(boxCollisionSphere);

% Get joint angles
q = Trials(TrialIndex).q;
% Get sampling time
Ts = mean(diff(Trials(TrialIndex).t));
% Get spheres
spheres = Trials(TrialIndex).humanModel.getSpheresStructureForAnimation();

% Animate
figure
Animate_nDOF_Spheres(q, L, spheres, 3*Ts);
