function [A_lat,B_lat] = LateralDynamics_Algebraic(FD, xbar)
%LATERAL MATRICES LINEARISED
%   Marty, Inga 25/05/2021
%   Marty       27/05/2021
%   Inga        29/05/2021

%%  Initialisations

m = FD.Inertia.m;
[Vt, alpha, beta] = AeroAngles(xbar);

Q = 0.5 * 1.225 * Vt^2;

CL = (m*FD.Inertia.g)/(Q*FD.Geom.S);
alpha = (CL - FD.Aero.CLo)/FD.Aero.CLa;

theta1 = alpha; %steady level flight ... no climb angle
u1 = Vt*cos(alpha);

%% Defining Matrix Variables
    %As defined in: Lateral modal approximations-algebraic Nelson
    
%%% Lateral directional derivatives

%beta - sideslip angle
Yb = (Q*FD.Geom.S*FD.Aero.Cyb)/m;
Nb = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cnb)/FD.Inertia.Izz;
Lb = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Clb)/FD.Inertia.Ixx;

%p - Roll
Yp = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cyp)/(2*m*u1);
Np = (Q*FD.Geom.S*FD.Geom.b^2*FD.Aero.Cnp)/(2*FD.Inertia.Izz*u1);
Lp = (Q*FD.Geom.S*FD.Geom.b^2*FD.Aero.Clp)/(2*FD.Inertia.Ixx*u1);

%r - yaw
Yr = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cyr)/(2*m*u1);
Nr = (Q*FD.Geom.S*FD.Geom.b^2*FD.Aero.Cnr)/(2*FD.Inertia.Izz*u1);
Lr = (Q*FD.Geom.S*FD.Geom.b^2*FD.Aero.Clr)/(2*FD.Inertia.Ixx*u1);

%da - aileron deflection
Yda = (Q*FD.Geom.S*FD.Aero.Cyda)/m;
Nda = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cnda)/FD.Inertia.Izz;
Lda = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Clda)/FD.Inertia.Ixx;

%da - rudder deflection
Ydr = (Q*FD.Geom.S*FD.Aero.Cydr)/m;
Ndr = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cndr)/FD.Inertia.Izz;
Ldr = (Q*FD.Geom.S*FD.Geom.b*FD.Aero.Cldr)/FD.Inertia.Ixx;

%% Defining Matrices
% [v,p,r,phi,psi]
A_lat = [Yb/u1, Yp/u1, Yr/u1-1, FD.Inertia.g*cos(theta1)/u1, 0;...
         Lb, Lp, Lr, 0, 0;...
         Nb, Np, Nr, 0, 0;...
         0,   1,  0, 0, 0;...
         0,   0,  1, 0, 0];

B_lat = [0, Ydr/u1;...
         Lda, Ldr;...
         Nda, Ndr;...
         0,    0;...
         0,    0];
      
end