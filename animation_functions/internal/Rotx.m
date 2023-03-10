function Rx = Rotx(theta)
%ROTX
%    RX = ROTX(THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-Feb-2022 17:12:33

%This function returns a matrix of rotation about the X-axis by a given angle.
t2 = cos(theta);
t3 = sin(theta);
Rx = reshape([1.0,0.0,0.0,0.0,t2,t3,0.0,-t3,t2],[3,3]);
