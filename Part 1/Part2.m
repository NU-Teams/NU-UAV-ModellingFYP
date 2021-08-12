%% setup
clear 
close all

%% Init

ft = 3.2808399;
kts = 1.94384449;

x_cg = 0.2922;
VT = 100/kts;
Altitude = 500/ft;
rho_ssl = 1.225;

FD = Initialisation(x_cg);
W = (FD.Inertia.m+110)*FD.Inertia.g;

[DensityRatio, Q] = FlowProperties(VT, Altitude);
rho = rho_ssl*DensityRatio;

Cno = -0.02;
Clo = -0.05;

%% a

% offsets
a.Co = [0; Clo; Cno];

% coefficients
a.Cb = [FD.Aero.Cyb FD.Aero.Cyda FD.Aero.Cydr;
        FD.Aero.Clb FD.Aero.Clda FD.Aero.Cldr;
        FD.Aero.Cnb FD.Aero.Cnda FD.Aero.Cndr];

% controls
a.sb = (a.Cb^-1)*(-a.Co);
disp(a.sb*180/pi)
%% b

% aileron and rudder
b.sb = [25  -25;
        25   25]*(pi/180);

b.Cb = [FD.Aero.Cyda FD.Aero.Cydr;
       FD.Aero.Clda FD.Aero.Cldr;
       FD.Aero.Cnda FD.Aero.Cndr];
  
b.Co = b.Cb*b.sb;

% beta and offsets
b.beta = -b.Co(1,:)/FD.Aero.Cyb;
b.Clo = -b.Co(2,:)-FD.Aero.Clb*b.beta;
b.Cno = -b.Co(3,:)-FD.Aero.Cnb*b.beta;

disp('b) Controls Full positive:')
disp('+-[Clo Cno]')
disp([b.Clo(1) b.Cno(1)])
disp('Controls positive/negative:')
disp('+-[Clo Cno]')
disp([b.Clo(2) b.Cno(2)])

%% c

CL = W/(Q*FD.Geom.S);

% aileron and rudder
c.Co = a.Co;
c.Cphi = [CL FD.Aero.Cyda FD.Aero.Cydr;
           0 FD.Aero.Clda FD.Aero.Cldr;
           0 FD.Aero.Cnda FD.Aero.Cndr];   
c.sphi = (c.Cphi^-1)*(-c.Co);
c.phi = c.sphi(1);
c.delta_a = c.sphi(2);
c.delta_r = c.sphi(3);

% elevator
long_in = [-FD.Aero.Cmo;
           CL-FD.Aero.CLo];
long_mat = [FD.Aero.Cma FD.Aero.Cmde;
            FD.Aero.CLa FD.Aero.CLde];
long_out = (long_mat^-1)*long_in;
alpha = long_out(1);
c.delta_e = long_out(2);

% drag
CD = FD.Aero.Cdo + FD.Aero.k*CL^2;
D = FD.Geom.S*Q*CD;
L = FD.Geom.S*Q*CL;
Cbs = C_y(alpha);
Aero_Force = Cbs*[-D; 0; -L];
FAx = Aero_Force(1);
Thrust = -FAx;
Power = FD.Prop.P_max*DensityRatio^0.7;
c.delta_T = Thrust*VT/(Power*FD.Prop.eta);

c.Inputs = [c.delta_T; c.delta_e*180/pi; c.delta_a*180/pi; c.delta_r*180/pi];
disp('c) phi [deg]')
disp(c.phi*180/pi)
disp('Inputs [deg]')
disp(c.Inputs)