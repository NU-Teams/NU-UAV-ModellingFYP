%% setup
clear 
close all

%% Init

ft = 3.2808399;
kts = 1.94384449;

x_cg = 0.2922;
VT = 300/kts;
Altitude = 500/ft;
rho_ssl = 1.225;

FD = Initialisation(x_cg);
W = (FD.Inertia.m+110)*FD.Inertia.g;

[DensityRatio, Q] = FlowProperties(VT, Altitude);
rho = rho_ssl*DensityRatio;

%% a

omega = 360*(pi/180)/30;
phi = atan(omega*VT/FD.Inertia.g);
n_z = 1/cos(phi);

psi_dot = omega; % yes

r = psi_dot*cos(phi);

CL = n_z*W/(Q*FD.Geom.S);

Trim_cond = [CL n_z phi*180/pi psi_dot*180/pi r*180/pi];
disp('a) Trim conditions')
disp('CL n_z phi psi_dot r [deg]')
disp(Trim_cond)
%% b

q = psi_dot*sin(phi);
p = 0; % ???

r_hat = r*(2*FD.Geom.b/VT);
q_hat = q*(2*FD.Geom.c/VT);

delta_a = (FD.Aero.Clr/FD.Aero.Clda)*r_hat;
delta_r = (FD.Aero.Cnr/FD.Aero.Cndr)*r_hat;

% Cm = Cm0 + Cma*alpha + Cmq*q_hat + CMde*delta_e;
% CL = CL0 + CLa*alpha + CLq*q_hat + CLde*delta_e;
%[Cm-Cmo-Cmq] = A*[alpha]
%[CL-CLo-CLq] = A*[delta_e]
long_in = [0-FD.Aero.Cmo-FD.Aero.Cmq*q_hat;
           CL-FD.Aero.CLo-FD.Aero.CLq*q_hat];
long_mat = [FD.Aero.Cma FD.Aero.Cmde;
            FD.Aero.CLa FD.Aero.CLde];
long_out = (long_mat^-1)*long_in;

alpha = long_out(1);
delta_e = long_out(2);

CD = FD.Aero.Cdo + FD.Aero.k*CL^2;
D = FD.Geom.S*Q*CD;
L = FD.Geom.S*Q*CL;

Cbs = C_y(alpha);

Aero_Force = Cbs*[-D; 0; -L];
FAx = Aero_Force(1);
Thrust = -FAx;

Power = FD.Prop.P_max*DensityRatio^0.7;
delta_T = Thrust*VT/(Power*FD.Prop.eta);

Inputs = [delta_T; delta_e*180/pi; delta_a*180/pi; delta_r*180/pi];

disp('p q r [deg]')
disp([p q r]*180/pi)
disp('b) Inputs')
disp(Inputs);

%% C

da_r = FD.Aero.Cnda/FD.Aero.Clda;
beta = (da_r*FD.Aero.Clr-FD.Aero.Cnr)/(FD.Aero.Cnb-da_r*FD.Aero.Clb);

beta_r = FD.Aero.Clb/FD.Aero.Cnb;
delta_a2 = r_hat*((beta_r*FD.Aero.Cnr-FD.Aero.Clr)/(FD.Aero.Clda-beta_r*FD.Aero.Cnda));
beta_r*180/pi
Inputs2 = [delta_T; delta_e*180/pi; delta_a2*180/pi; 0];
disp('c) Inputs')
disp(Inputs2);

%% D

lat_stability = beta_r*(FD.Aero.Cnr/FD.Aero.Clr);
disp('d) Is it laterally stable?')
disp(lat_stability)
if lat_stability < 1
    disp('Unstable')
elseif lat_stability > 1
    disp('Stable')
else
    disp('Neutral')
end






