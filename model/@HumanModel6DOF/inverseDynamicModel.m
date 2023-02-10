% Inverse Dynamic Model using Newton-Euler Algorithm
% Robot with rigid joints and fixed base


% Geometric parameters
% j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
% 1       0       2       0       0       0       0       0       0       0       
% 2       1       0       1       0       0       0       0       q1      0       
% 3       2       0       1       0       0       0       L1      q2      0       
% 4       3       0       1       0       0       0       L2      q3      0       
% 5       4       0       1       0       0       0       L3      q4      0       
% 6       5       0       1       0       0       0       L4      q5      0       
% 7       6       0       1       0       0       0       L5      q6      0       
% 8       7       2       0       0       0       0       L6      0       0       
% 9       5       2       0       0       0       0       LT2H    0       0       

% Dynamic inertia parameters
% j       XX      XY      XZ      YY      YZ      ZZ      MX      MY      MZ      M       IA      
% 1       0       0       0       0       0       ZZFOOT  MXFOOT  MYFOOT  MZFOOT  MFOOT   0       
% 2       0       0       0       0       0       ZZ1     MX1     MY1     MZ1     M1      0       
% 3       0       0       0       0       0       ZZ2     MX2     MY2     MZ2     M2      0       
% 4       0       0       0       0       0       ZZ3     MX3     MY3     MZ3     M3      0       
% 5       0       0       0       0       0       ZZ4     MX4     MY4     MZ4     M4      0       
% 6       0       0       0       0       0       ZZ5     MX5     MY5     MZ5     M5      0       
% 7       0       0       0       0       0       ZZ6     MX6     MY6     MZ6     M6      0       
% 8       0       0       0       0       0       ZZHAND  MXHAND  MYHAND  MZHAND  MHAND   0       
% 9       0       0       0       0       0       ZZHEAD  MXHEAD  MYHEAD  MZHEAD  MHEAD   0       

% External forces and joint parameters
% j       FX      FY      FZ      CX      CY      CZ      FS      FV      QP      QDP     GAM     eta     k       
% 1       FXFOOT  FYFOOT  FZFOOT  CXFOOT  CYFOOT  CZFOOT  0       0       0       0       0       0       0       
% 2       0       0       0       0       0       0       0       0       dq1     ddq1    TAU1    0       0       
% 3       0       0       0       0       0       0       0       0       dq2     ddq2    TAU2    0       0       
% 4       0       0       0       0       0       0       0       0       dq3     ddq3    TAU3    0       0       
% 5       0       0       0       0       0       0       0       0       dq4     ddq4    TAU4    0       0       
% 6       0       0       0       0       0       0       0       0       dq5     ddq5    TAU5    0       0       
% 7       0       0       0       0       0       0       0       0       dq6     ddq6    TAU6    0       0       
% 8       FXHAND  FYHAND  FZHAND  CXHAND  CYHAND  CZHAND  0       0       0       0       0       0       0       
% 9       0       0       0       0       0       0       0       0       0       0       0       0       0       

% Base velicities parameters
% axis    W0      WP0     V0      VP0     G       
% X       0       0       0       0       0       
% Y       0       0       0       0       GY      
% Z       0       0       0       0       0      

function [tau, f_grf] = inverseDynamicModel(obj,q,dq,ddq,fFOOT,fHAND)
	% Preliminary: Assign Trajectory Related Variables
	q1 = q(1, :);
	q2 = q(2, :);
	q3 = q(3, :);
	q4 = q(4, :);
	q5 = q(5, :);
	q6 = q(6, :);
	dq1 = dq(1, :);
	dq2 = dq(2, :);
	dq3 = dq(3, :);
	dq4 = dq(4, :);
	dq5 = dq(5, :);
	dq6 = dq(6, :);
	ddq1 = ddq(1, :);
	ddq2 = ddq(2, :);
	ddq3 = ddq(3, :);
	ddq4 = ddq(4, :);
	ddq5 = ddq(5, :);
	ddq6 = ddq(6, :);
	
	% Preliminary: Assign External Force Related Variables
	FXFOOT = fFOOT(1, :);
	FYFOOT = fFOOT(2, :);
	FZFOOT = fFOOT(3, :);
	CXFOOT = fFOOT(4, :);
	CYFOOT = fFOOT(5, :);
	CZFOOT = fFOOT(6, :);
	
	FXHAND = fHAND(1, :);
	FYHAND = fHAND(2, :);
	FZHAND = fHAND(3, :);
	CXHAND = fHAND(4, :);
	CYHAND = fHAND(5, :);
	CZHAND = fHAND(6, :);
	
	% Preliminary: Assign Model Realted Variables
	GY = obj.Gravity;
	
	L1 = obj.L(1);
	L2 = obj.L(2);
	L3 = obj.L(3);
	L4 = obj.L(4);
	L5 = obj.L(5);
	L6 = obj.L(6);
	LT2H = obj.L(4);
	
	MFOOT = obj.MFOOT;
	M1 = obj.M(1);
	M2 = obj.M(2);
	M3 = obj.M(3);
	M4 = obj.M(4);
	M5 = obj.M(5);
	M6 = obj.M(6);
	MHAND = obj.MHAND;
	MHEAD = obj.MHEAD;
	
	MXFOOT = obj.CoMFOOT(1,1) .* obj.MFOOT;
	MX1 = obj.CoM(1,1) .* obj.M(1);
	MX2 = obj.CoM(1,2) .* obj.M(2);
	MX3 = obj.CoM(1,3) .* obj.M(3);
	MX4 = obj.CoM(1,4) .* obj.M(4);
	MX5 = obj.CoM(1,5) .* obj.M(5);
	MX6 = obj.CoM(1,6) .* obj.M(6);
	MXHAND = obj.CoMHAND(1,1) .* obj.MHAND;
	MXHEAD = obj.CoMHEAD(1,1) .* obj.MHEAD;
	
	MYFOOT = obj.CoMFOOT(2,1) .* obj.MFOOT;
	MY1 = obj.CoM(2,1) .* obj.M(1);
	MY2 = obj.CoM(2,2) .* obj.M(2);
	MY3 = obj.CoM(2,3) .* obj.M(3);
	MY4 = obj.CoM(2,4) .* obj.M(4);
	MY5 = obj.CoM(2,5) .* obj.M(5);
	MY6 = obj.CoM(2,6) .* obj.M(6);
	MYHAND = obj.CoMHAND(2,1) .* obj.MHAND;
	MYHEAD = obj.CoMHEAD(2,1) .* obj.MHEAD;
    	
	MZFOOT = 0;
	MZ1 = 0;
	MZ2 = 0;
	MZ3 = 0;
	MZ4 = 0;
	MZ5 = 0;
	MZ6 = 0;
	MZHAND = 0;
	MZHEAD = 0;
		
	ZZFOOT = obj.IzzFOOT;
	ZZ1 = obj.Izz(1);
	ZZ2 = obj.Izz(2);
	ZZ3 = obj.Izz(3);
	ZZ4 = obj.Izz(4);
	ZZ5 = obj.Izz(5);
	ZZ6 = obj.Izz(6);
	ZZHAND = obj.IzzHAND;
	ZZHEAD = obj.IzzHEAD;
	
	% Equations:
	S_q1 = sin(q1);
	C_q1 = cos(q1);
	C_q2 = cos(q2);
	S_q2 = sin(q2);
	C_q3 = cos(q3);
	S_q3 = sin(q3);
	C_q4 = cos(q4);
	S_q4 = sin(q4);
	S_q5 = sin(q5);
	C_q5 = cos(q5);
	S_q6 = sin(q6);
	C_q6 = cos(q6);
	DV62 = dq1.^2;
	VP12 = -GY.*S_q1;
	VP22 = -C_q1.*GY;
	W33 = dq1 + dq2;
	WP33 = ddq1 + ddq2;
	DV63 = W33.^2;
	VSP13 = -DV62.*L1 + VP12;
	VSP23 = L1.*ddq1 + VP22;
	VP13 = C_q2.*VSP13 + S_q2.*VSP23;
	VP23 = C_q2.*VSP23 - S_q2.*VSP13;
	W34 = W33 + dq3;
	WP34 = WP33 + ddq3;
	DV64 = W34.^2;
	VSP14 = -DV63.*L2 + VP13;
	VSP24 = L2.*WP33 + VP23;
	VP14 = C_q3.*VSP14 + S_q3.*VSP24;
	VP24 = C_q3.*VSP24 - S_q3.*VSP14;
	W35 = W34 + dq4;
	WP35 = WP34 + ddq4;
	DV65 = W35.^2;
	VSP15 = -DV64.*L3 + VP14;
	VSP25 = L3.*WP34 + VP24;
	VP15 = C_q4.*VSP15 + S_q4.*VSP25;
	VP25 = C_q4.*VSP25 - S_q4.*VSP15;
	W36 = W35 + dq5;
	WP36 = WP35 + ddq5;
	DV66 = W36.^2;
	VSP16 = -DV65.*L4 + VP15;
	VSP26 = L4.*WP35 + VP25;
	VP16 = C_q5.*VSP16 + S_q5.*VSP26;
	VP26 = C_q5.*VSP26 - S_q5.*VSP16;
	W37 = W36 + dq6;
	WP37 = WP36 + ddq6;
	DV67 = W37.^2;
	VSP17 = -DV66.*L5 + VP16;
	VSP27 = L5.*WP36 + VP26;
	VP17 = C_q6.*VSP17 + S_q6.*VSP27;
	VP27 = C_q6.*VSP27 - S_q6.*VSP17;
	VSP18 = -DV67.*L6 + VP17;
	VSP28 = L6.*WP37 + VP27;
	VSP19 = -DV65.*LT2H + VP15;
	VSP29 = LT2H.*WP35 + VP25;
	F21 = -GY.*MFOOT;
	F12 = -DV62.*MX1 + M1.*VP12 - MY1.*ddq1;
	F22 = -DV62.*MY1 + M1.*VP22 + MX1.*ddq1;
	PSI32 = ZZ1.*dq1;
	No32 = ZZ1.*ddq1;
	F13 = -DV63.*MX2 + M2.*VP13 - MY2.*WP33;
	F23 = -DV63.*MY2 + M2.*VP23 + MX2.*WP33;
	PSI33 = W33.*ZZ2;
	No33 = WP33.*ZZ2;
	F14 = -DV64.*MX3 + M3.*VP14 - MY3.*WP34;
	F24 = -DV64.*MY3 + M3.*VP24 + MX3.*WP34;
	PSI34 = W34.*ZZ3;
	No34 = WP34.*ZZ3;
	F15 = -DV65.*MX4 + M4.*VP15 - MY4.*WP35;
	F25 = -DV65.*MY4 + M4.*VP25 + MX4.*WP35;
	PSI35 = W35.*ZZ4;
	No35 = WP35.*ZZ4;
	F16 = -DV66.*MX5 + M5.*VP16 - MY5.*WP36;
	F26 = -DV66.*MY5 + M5.*VP26 + MX5.*WP36;
	PSI36 = W36.*ZZ5;
	No36 = WP36.*ZZ5;
	F17 = -DV67.*MX6 + M6.*VP17 - MY6.*WP37;
	F27 = -DV67.*MY6 + M6.*VP27 + MX6.*WP37;
	PSI37 = W37.*ZZ6;
	No37 = WP37.*ZZ6;
	F18 = -DV67.*MXHAND + MHAND.*VSP18 - MYHAND.*WP37;
	F28 = -DV67.*MYHAND + MHAND.*VSP28 + MXHAND.*WP37;
	PSI38 = W37.*ZZHAND;
	No38 = WP37.*ZZHAND;
	F19 = -DV65.*MXHEAD + MHEAD.*VSP19 - MYHEAD.*WP35;
	F29 = -DV65.*MYHEAD + MHEAD.*VSP29 + MXHEAD.*WP35;
	PSI39 = W35.*ZZHEAD;
	No39 = WP35.*ZZHEAD;
	N19 = -MZHEAD.*VSP29;
	N29 = MZHEAD.*VSP19;
	N39 = MXHEAD.*VSP29 - MYHEAD.*VSP19 + No39;
	E18 = F18 + FXHAND;
	E28 = F28 + FYHAND;
	N18 = CXHAND - MZHAND.*VSP28;
	N28 = CYHAND + MZHAND.*VSP18;
	N38 = CZHAND + MXHAND.*VSP28 - MYHAND.*VSP18 + No38;
	E17 = E18 + F17;
	E27 = E28 + F27;
	N17 = -MZ6.*VP27 + N18;
	N27 = -FZHAND.*L6 + MZ6.*VP17 + N28;
	N37 = E28.*L6 + MX6.*VP27 - MY6.*VP17 + N38 + No37;
	FDI17 = C_q6.*E17 - E27.*S_q6;
	FDI27 = C_q6.*E27 + E17.*S_q6;
	E16 = F16 + FDI17;
	E26 = F26 + FDI27;
	N16 = C_q6.*N17 - MZ5.*VP26 - N27.*S_q6;
	N26 = C_q6.*N27 - FZHAND.*L5 + MZ5.*VP16 + N17.*S_q6;
	N36 = FDI27.*L5 + MX5.*VP26 - MY5.*VP16 + N37 + No36;
	FDI16 = C_q5.*E16 - E26.*S_q5;
	FDI26 = C_q5.*E26 + E16.*S_q5;
	E15 = F15 + F19 + FDI16;
	E25 = F25 + F29 + FDI26;
	N15 = C_q5.*N16 - MZ4.*VP25 + N19 - N26.*S_q5;
	N25 = C_q5.*N26 - FZHAND.*L4 + MZ4.*VP15 + N16.*S_q5 + N29;
	N35 = F29.*LT2H + FDI26.*L4 + MX4.*VP25 - MY4.*VP15 + N36 + N39 + No35;
	FDI15 = C_q4.*E15 - E25.*S_q4;
	FDI25 = C_q4.*E25 + E15.*S_q4;
	E14 = F14 + FDI15;
	E24 = F24 + FDI25;
	N14 = C_q4.*N15 - MZ3.*VP24 - N25.*S_q4;
	N24 = C_q4.*N25 - FZHAND.*L3 + MZ3.*VP14 + N15.*S_q4;
	N34 = FDI25.*L3 + MX3.*VP24 - MY3.*VP14 + N35 + No34;
	FDI14 = C_q3.*E14 - E24.*S_q3;
	FDI24 = C_q3.*E24 + E14.*S_q3;
	E13 = F13 + FDI14;
	E23 = F23 + FDI24;
	N13 = C_q3.*N14 - MZ2.*VP23 - N24.*S_q3;
	N23 = C_q3.*N24 - FZHAND.*L2 + MZ2.*VP13 + N14.*S_q3;
	N33 = FDI24.*L2 + MX2.*VP23 - MY2.*VP13 + N34 + No33;
	FDI13 = C_q2.*E13 - E23.*S_q2;
	FDI23 = C_q2.*E23 + E13.*S_q2;
	E12 = F12 + FDI13;
	E22 = F22 + FDI23;
	N12 = C_q2.*N13 - MZ1.*VP22 - N23.*S_q2;
	N22 = C_q2.*N23 - FZHAND.*L1 + MZ1.*VP12 + N13.*S_q2;
	N32 = FDI23.*L1 + MX1.*VP22 - MY1.*VP12 + N33 + No32;
	FDI12 = C_q1.*E12 - E22.*S_q1;
	FDI22 = C_q1.*E22 + E12.*S_q1;
	E11 = FDI12 + FXFOOT;
	E21 = F21 + FDI22 + FYFOOT;
	E31 = FZFOOT + FZHAND;
	N11 = CXFOOT + C_q1.*N12 + GY.*MZFOOT - N22.*S_q1;
	N21 = CYFOOT + C_q1.*N22 + N12.*S_q1;
	N31 = CZFOOT - GY.*MXFOOT + N32;
	GAM1 = 0;
	GAM2 = N32;
	GAM3 = N33;
	GAM4 = N34;
	GAM5 = N35;
	GAM6 = N36;
	GAM7 = N37;
	GAM8 = 0;
	GAM9 = 0;
	E10 = E11;
	E20 = E21;
	E30 = E31;
	N10 = N11;
	N20 = N21;
	N30 = N31;
	
	tau = [GAM2; GAM3; GAM4; GAM5; GAM6; GAM7];
	f_grf = -[E10; E20; E30; N10; N20; N30];
end