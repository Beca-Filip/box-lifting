clear all;
close all;
clc;

%% Casadi SX quadratic function
n = 20;
x = casadi.SX.sym('x', n, 1);
Q = casadi.SX.sym('Q', n, n);
r = casadi.SX.sym('r', n, 1);

f = .5 * x.' * Q * x + r.' * x;
gradf = jacobian(f, x).';
hessf = hessian(f, x);

% Show the SX versions
f
gradf
hessf

%% Casadi MX quadratic function
n = 20;
x = casadi.MX.sym('x', n, 1);
Q = casadi.MX.sym('Q', n, n);
r = casadi.MX.sym('r', n, 1);

f = .5 * x.' * Q * x + r.' * x;
gradf = jacobian(f, x).';
hessf = hessian(f, x);

% Show the SX versions
f
gradf
hessf