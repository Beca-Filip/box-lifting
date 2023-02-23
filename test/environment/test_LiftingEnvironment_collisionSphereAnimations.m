clear all;
close all;
clc;

% Load lifting trials
Trials = importdata("../../processed_data/Lifting/kinematically_calibrated_lifting_with_environment.mat", "Trials");

% Initialize trial index
TrialIndex = 1;
TrialsWithIntersections = [];

for TrialIndex = 1 : length(Trials)
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
% Create table collision sphere
tableCollisionSphere = Trials(TrialIndex).liftingEnvironment.createTableCollisionSphere();

% Add the collision spheres to the human model
Trials(TrialIndex).humanModel = Trials(TrialIndex).humanModel.addCollisionSphere(boxCollisionSphere);
Trials(TrialIndex).humanModel = Trials(TrialIndex).humanModel.addCollisionSphere(tableCollisionSphere);

% Get joint angles
q = Trials(TrialIndex).q;
% Get sampling time
Ts = mean(diff(Trials(TrialIndex).t));
% Get spheres
spheres = Trials(TrialIndex).humanModel.getSpheresStructureForAnimation();

% Animate
TrialIndex
figure
Animate_nDOF_Spheres(q, L, spheres, round(1.5*Ts, 2));
pause(round(1.5*Ts, 2) * Trials(TrialIndex).nbSamples + 1);
close

bodySpheres = find(contains([Trials(TrialIndex).humanModel.CS.Name] ,"Link", "IgnoreCase", true));
boxSphere = find(contains([Trials(TrialIndex).humanModel.CS.Name] ,"Box", "IgnoreCase", true));
tableSpheres = find(contains([Trials(TrialIndex).humanModel.CS.Name] ,"Table", "IgnoreCase", true));

D = Trials(TrialIndex).humanModel.collisionSpheresDistances(Trials(TrialIndex).q);

distBodyToBox = cell2mat(D(bodySpheres, boxSphere));
distBoxToTable = cell2mat(D(boxSphere, tableSpheres).');

if any(distBoxToTable <= 0, 'all')
    TrialsWithIntersections = [TrialIndex];
end

end
