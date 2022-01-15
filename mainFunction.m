function output = mainFunction()

% TODO (PropForces):
%   implement user input for propulsion axes and distance from CoG

%% Setup
clear
close all
clc

addpath("FlightSim");
addpath("FlightSim\CoordinateTransformation");
addpath("FlightSim\Dynamics");
addpath("FlightSim\Forces");
addpath("FlightSim\LinearAnalysis");

addpath("FlightSim\Output");
addpath("FlightSim\Output\data");
addpath("FlightSim\Output\figures");

addpath("FlightSim\Parameters");

addpath("savedFiles\temp");
%addpath("savedFiles\savedAircraft_ProfilePhotos");

load("appParameters", 'aircraft', 'operation', 'environment', 'simulation');


T = 0:simulation.timeStep:simulation.timeSpan;

%% Configuration

% Find the Trim States
[Xbar, Ubar] = Trim(aircraft, operation, environment);

% Generate the Input Array
[U, ~] = Controls(Xbar, Ubar, T, aircraft, operation, environment);


%% Eigen Analysis

[aircraft.A, aircraft.B] = Linearise(Xbar, Ubar, aircraft, environment);

[~, ~, aircraft.Long.A, aircraft.Long.B]	= LongMatrixDecouple(aircraft.A, aircraft.B);
[~, ~, aircraft.Lat.A, aircraft.Lat.B]      = LatMatrixDecouple(aircraft.A, aircraft.B);

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
[X_lin, ~] = IntegrateLinearDynamics(T, X0, U, aircraft, environment);

% Non-Linear Integration
[X_nl, ~] = IntegrateDynamics(X0, U, T, aircraft, environment);


%% Plot States

output.Transient.T      = T;
output.Transient.X_lin	= X_lin;
output.Transient.X_nl	= X_nl;
output.Transient.U      = U;

output.LinearAnalysis.A             = aircraft.A;
output.LinearAnalysis.B             = aircraft.B;
output.LinearAnalysis.eigValues 	= eigValues;
output.LinearAnalysis.eigVectors	= eigVectors;
output.LinearAnalysis.eigDiagonal   = eigDiagonal;
output.LinearAnalysis.natFreq       = natFreq;
output.LinearAnalysis.dampFactor	= dampFactor;

output.LinearAnalysis.Stability     = Stability;

output.MilSpec                      = MilSpec;

%Figures = PlotTransientResponse(T, X_lin, X_nl, U);


%% Save Data 
% 
% save('Output\data\temp\A_long.mat', 'A_long')
% save('Output\data\temp\B_long.mat', 'B_long')
% save('Output\data\temp\EgnVector_long.mat', 'EgnVector_long')
% save('Output\data\temp\EgnValue_long.mat', 'EgnValue_long')
% save('Output\data\temp\Wn_long.mat', 'Wn_long')
% save('Output\data\temp\zeta_long.mat', 'zeta_long')
% 
% save('Output\data\temp\A_lat.mat.mat', 'A_lat')
% save('Output\data\temp\B_lat.mat.mat', 'B_lat')
% save('Output\data\temp\EgnVector_lat.mat', 'EgnVector_lat')
% save('Output\data\temp\EgnValue_lat.mat', 'EgnValue_lat')
% save('Output\data\temp\Wn_lat.mat', 'Wn_lat')
% save('Output\data\temp\zeta_lat.mat', 'zeta_lat')
% 
% save('Output\data\temp\load_alpha.mat', 'load_alpha')
% saveas(Figure_states, 'Output\figures\temp\response.svg')

%}
end