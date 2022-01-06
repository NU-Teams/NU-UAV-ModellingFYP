%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AERO3000 Assignment 4 - Linearised Flight Simulator
% Jason Iredale
% Inga Leinasars
% Ashleigh Rattray
% Martin Shannon
% June 2021
% 
% NU UAV Flight Simulator
% Shane Pomfrett
% Jason Iredale
% September 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup
clear
close all
clc

addpath("Parameters");
addpath("Parameters\Aircraft");
addpath("Parameters\Environment");

addpath("CoordinateTransformation");
addpath("Dynamics");
addpath("Forces");
addpath("LinearAnalysis");

addpath("Output");
addpath("Output\data");
addpath("Output\figures");

T_final = 10;
timeStep = 0.01;
T = 0:timeStep:T_final;


%% Configuration

% Initilise PC-21 Flight data
[FD, enviro, config_str] = Initialisation();

% Find the Trim States
[Xbar, Ubar] = Trim(FD);

% Generate the Input Array
[U, U_impulse, manoeuvre_str] = Controls(Xbar, Ubar, T, FD);


%% Eigen Analysis

[FD.A, FD.B] = Linearise(Xbar, Ubar, FD);

[~, ~, FD.Long.A, FD.Long.B] = LongMatrixDecouple(FD.A, FD.B);
[~, ~, FD.Lat.A, FD.Lat.B] = LatMatrixDecouple(FD.A, FD.B, Xbar);

% compute eig vals/vectors of matrix A (eig and damp)
[V,E,Wn,zeta] = eig_analysis(FD.A);

% eigenvectors and eigenvalues
    %PlotStability(T, FD, 'long')
    %PlotStability(T, FD, 'lat')

% Mil-Spec
    %[MilSpecFigure, load_alpha] = PlotMilSpec(FD, Vt);


%% Linear Integration for Time Response Analysis
% linearize equilibrium states and inputs

% Establish the Initial Conditions
X0 = Xbar;

% Linear Dynamics
[X_lin, X_dot_lin] = IntegrateLinearDynamics(T, X0, U, FD);

% Non-Linear Integration
[X_nl, X_dot_nl] = IntegrateDynamics(X0, U, T, FD);


%% Plot States

Figures = PlotTransientResponse(T, X_lin, X_nl, U);


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

