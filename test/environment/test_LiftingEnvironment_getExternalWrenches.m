clear all;
close all;
clc;

H = HumanModel6DOF();
LE =  LiftingEnvironment();
LE.WristToBoxGripPointVector = zeros(2, 1);
LE.BoxMass = 1;

q = [pi/2; zeros(3, 1); -3*pi/4; 0];
dq = zeros(6, 1);
ddq = zeros(6, 1);

figure
Animate_nDOF(q, H.L, 0.01);

fFOOT = zeros(6, 1);
fHAND = zeros(6, 1);
[tau, f_grf] = H.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND);
tau
f_grf

fHAND = LE.getExternalWrenches(q, H.Gravity);
fHAND
% fHAND = [0; 9.81; zeros(4, 1)];
[tau, f_grf] = H.inverseDynamicModel(q, dq, ddq, fFOOT, fHAND);
tau
f_grf