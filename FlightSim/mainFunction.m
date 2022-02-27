function output = mainFunction(parameters)

% TODO:
% wrap in current draw / fuel into dynamics
% display delta_T, power and omega
% allow blackbox data for aerodynamic coefficients
% propeller inertia?

%% Setup
close all
clc

addpath("FlightSim");
addpath("FlightSim/CoordinateTransformation");
addpath("FlightSim/Dynamics");
addpath("FlightSim/Forces");
addpath("FlightSim/LinearAnalysis");

addpath("FlightSim/Parameters");


% Unpack
aircraft    = parameters.aircraft;
operation   = parameters.operation;
environment = parameters.environment;
simulation  = parameters.simulation;

T = 0:simulation.timeStep:simulation.timeSpan;

% convert input parameters to SI
kts = 1.944;
deg = 180/pi;
ft  = 3.28084;

aircraft.CtrlLim.Lwr(2:5)       = aircraft.CtrlLim.Lwr(2:5)/deg;
aircraft.CtrlLim.Upr(2:5)       = aircraft.CtrlLim.Upr(2:5)/deg;
environment.Wind.speed          = environment.Wind.speed/kts;
environment.Wind.bearing        = environment.Wind.bearing/deg;
operation.Trim.airspeed         = operation.Trim.airspeed/kts;
operation.Trim.bearing          = operation.Trim.bearing/deg;
operation.Trim.beta             = operation.Trim.beta/deg;
operation.Trim.altitude         = operation.Trim.altitude/(-ft);
operation.latitude0             = operation.latitude0/deg;
operation.longitude0            = operation.longitude0/deg;
operation.manualControls(2:5,:) = operation.manualControls(2:5,:)/deg;


%% Configuration

% Find the Trim States
[Xbar, Ubar] = Trim(aircraft, operation, environment);

% Generate the Input Array
U = controls(aircraft, operation, Ubar);

%% Eigen Analysis

[aircraft.A, aircraft.B] = Linearise(Xbar, Ubar, aircraft, environment);

[~, ~, aircraft.Long.A, aircraft.Long.B]	= LongMatrixDecouple(aircraft.A, aircraft.B);
[~, ~, aircraft.Lat.A , aircraft.Lat.B ]    = LatMatrixDecouple(aircraft.A , aircraft.B);

% compute eig vals/vectors of matrix A (eig and damp)
[eigValues, eigVectors, eigDiagonal, natFreq, dampFactor] = eig_analysis(aircraft.A);

% eigenvectors and eigenvalues
Stability = PlotStability(T, aircraft);

% Mil-Spec
MilSpec = PlotMilSpec(Xbar, aircraft, environment);


%% Linear Integration for Time Response Analysis
% linearize equilibrium states and inputs

% Establish the Initial Conditions
X0 = Xbar;

% Linear Dynamics
[X_lin, ~]          = IntegrateLinearDynamics(T, X0, U, aircraft, environment);

% Non-Linear Integration
[X_nl, X_dot_nl]    = IntegrateDynamics(X0, U, T, aircraft, operation, environment);


%% Pack to output

output.Transient.T                  = T;
output.Transient.X_lin              = X_lin;
output.Transient.X_nl               = X_nl;
output.Transient.X_dot_nl           = X_dot_nl;
output.Transient.U                  = U;

output.LinearAnalysis.A             = aircraft.A;
output.LinearAnalysis.B             = aircraft.B;
output.LinearAnalysis.eigValues 	= eigValues;
output.LinearAnalysis.eigVectors	= eigVectors;
output.LinearAnalysis.eigDiagonal   = eigDiagonal;
output.LinearAnalysis.natFreq       = natFreq;
output.LinearAnalysis.dampFactor	= dampFactor;

output.LinearAnalysis.Stability     = Stability;

output.MilSpec                      = MilSpec;


end