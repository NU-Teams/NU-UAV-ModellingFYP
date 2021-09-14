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

addpath("LinearDynamics");
addpath("NonLinearDynamics");

T_final = 10;
timeStep = 0.01;
T = 0:timeStep:T_final;

%% Configuration
% Print Config Info
[x_cg, mass, Vt, alt, config_str] = configuration();

% Initilise PC-21 Flight data
FD = Initialisation(x_cg, mass);
%FD = PC9_Flight_Data_2021();

% Find the Trim States
[Xbar, Ubar] = Trim(Vt, alt, FD);

% Generate the Input Array
[U, U_impulse, manoeuvre_str] = Controls(Xbar, Ubar, T, FD);


%% Eigen Analysis

[FD.A, FD.B]            = Linearise(Xbar, U(:,1), FD);

[~, ~, A_Long_5, B_Long_5] = LongMatrixDecouple(FD.A, FD.B);
[~, ~, A_Lat_5, B_Lat_5] = LatMatrixDecouple(FD.A, FD.B, Xbar);

%for each config & axis:
    %   compute eig vals/vectors of matrix A (eig and damp)
    [V_long,E_long,Wn_long,zeta_long] = eig_analysis(A_Long_5);
    [V_lat,E_lat,Wn_lat,zeta_lat] = eig_analysis(A_Lat_5);


%% Plotting
% eigenvectors and eigenvalues
%PlotStability(T, FD, 'long')
%PlotStability(T, FD, 'lat')

% Mil-Spec
%[MilSpecFigure, load_alpha] = PlotMilSpec(FD, Vt);


%% Linear Integration for Time Response Analysis
% linearize equilibrium states and inputs

% Find the Initial Conditions
X0 = Xbar;
% Linear Dynamics
[X_lin, X_dot_lin] = Integration(T, X0, 'both',  U, FD);
% Non-Linear Integration
[X_nl, X_dot_nl] = Integrate(X0, U, T, FD);

%% Plot States

Figures = PlotData(T, X_lin', X_nl');

%% Save Data

% A_long = FD.Long.A;
% B_long = FD.Long.B;
% A_lat = FD.Lat.A;
% B_lat = FD.Lat.B;
% 
% [EgnVector_long, EgnValue_long, Wn_long, zeta_long] = eig_analysis(FD.Long.A);
% [EgnVector_lat , EgnValue_lat , Wn_lat , zeta_lat] = eig_analysis(FD.Lat.A);
% 
% 
% save('data\temp\A_long.mat', 'A_long')
% save('data\temp\B_long.mat', 'B_long')
% save('data\temp\EgnVector_long.mat', 'EgnVector_long')
% save('data\temp\EgnValue_long.mat', 'EgnValue_long')
% save('data\temp\Wn_long.mat', 'Wn_long')
% save('data\temp\zeta_long.mat', 'zeta_long')
% 
% save('data\temp\A_lat.mat.mat', 'A_lat')
% save('data\temp\B_lat.mat.mat', 'B_lat')
% save('data\temp\EgnVector_lat.mat', 'EgnVector_lat')
% save('data\temp\EgnValue_lat.mat', 'EgnValue_lat')
% save('data\temp\Wn_lat.mat', 'Wn_lat')
% save('data\temp\zeta_lat.mat', 'zeta_lat')
% 
% save('data\temp\load_alpha.mat', 'load_alpha')
% saveas(Figure_states, 'figures\temp\response.svg')

