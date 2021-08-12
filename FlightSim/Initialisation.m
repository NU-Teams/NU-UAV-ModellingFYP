%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [ FD ] = Initialisation(x_cg)
% This function generates and returns the flight data structure which
% defines the aircraft data and aerodynamic derivatives.
%
% Aircraft data and aerodynamic derivatives  - CG 22% mac.
% Flight config = clean_xx - flaps deflected 0 deg, clean configuration
% 
% (c) Peter W. Gibbens & S. Dumble, 1 March, 2011.
% (Updated) Peter W. Gibbens, 29 April, 2021.
% (Updated) Martin D. Shannon, 30 April, 2021
% (Updated) Jason Iredale, 22 May 2021.
%   --> I added Assessment 2 solutions to generalise the x_cg location.
% (Updated) Inga, Ash & Marty re-calculated Cnb for the new cg location.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ FD ] = Initialisation(x_cg, mass)

% Note: FD --> FlightData

%% Inertial Data

% Assuming Similar Mass distribution to the PC/9, ratio of inertias given
% by: AC_mass_PC21/AC_mass_PC9 
Mratio = 2650/2087;
% Ratio of wing spans to better estimate Ixx
WSratio = 9.11/10.19;
% Ratio of body lengths to better estimate Iyy
BLratio = 11.23/10.18;
%ratio of airfcraft heights to better estimate Izz
HLratio = 3.75/3.26;
% Ixz is a product of inertia

% --> Inertia_PC21 = Mratio*OtherRatio*Inertia_PC9

FD.Inertia.g = 9.81;                        % Gravity Constant
FD.Inertia.m = 2650;                        % Aircraft Mass (kg)
FD.Inertia.Ixx = 5066*Mratio*WSratio^1;     % Aircraft Moments of Inertia (kg.m^2)
FD.Inertia.Iyy = 6578*Mratio*BLratio^1;     % Aircraft Moments of Inertia (kg.m^2)
FD.Inertia.Izz = 10975*Mratio*HLratio^1;    % Aircraft Moments of Inertia (kg.m^2)
FD.Inertia.Ixz = 203*Mratio;                % Aircraft Product of Inertia (kg.m^2)

%% Geometric Data
FD.Geom.S = 15.22;                          % Platform Area (m^2)
FD.Geom.c = 2.32;                           % Chord Length (m)
FD.Geom.b = 9.11;                           % Wing Span (m)

%% Propulsion Data
% Source:
% <http://www.mathworks.com https://en.wikipedia.org/wiki/Pratt_%26_Whitney_Canada_PT6#Specifications_(PT6A-6)>

FD.Prop.P_max = 1200000;                    % Maximum engine power (Watts)
FD.Prop.eta = 0.8;                          % Propeller efficiency

%% Control Data
DtoR = pi/180;

FD.CtrlLim.Lwr = [0;            % Throttle range (Fraction)
                                  -25*DtoR;     % Elevator range (rad)
                                  -25*DtoR;     % Aileron range (rad)
                                  -25*DtoR];    % Rudder range (rad)
FD.CtrlLim.Upr = [1;            % Throttle range (Fraction)
                                  25*DtoR;      % Elevator range (rad)
                                  25*DtoR;      % Aileron range (rad)
                                  25*DtoR];     % Rudder range (rad)

%% Aerodynamic Data 

% Zero-Lift Angle
FD.Aero.alpha_o = -0.0249;      % Calculated in Assignment 2 -0.0249 rads

% Drag Coefficients
FD.Aero.Cdo    =  0.023;        % CD = CD0 + k*CL^2, CD0 = 0.023
FD.Aero.k      =  0.0663;       % k = 1/(pi*AR*e)

% As long as you can justify it its fine!!!
% Lift Coefficients
FD.Aero.CLa  =  4.871;          % a as calculated in assignment 2
FD.Aero.CLq  =  9.771;          % CL/turn rate deriv of pitch rate in terms of lift
FD.Aero.CLad = -1.987;          % Left the same
FD.Aero.CLde =  0.532;          % elevator deflection very similar thus unchanged.
FD.Aero.CLo  = -FD.Aero.CLa*FD.Aero.alpha_o; %offset


% Side Force Coefficients
FD.Aero.Cyb  = -0.712;
FD.Aero.Cybd = -0.0032;         % Left the same
FD.Aero.Cyp  = -0.205;
FD.Aero.Cyr  =  0.542;
FD.Aero.Cyda =  0.000;
FD.Aero.Cydr =  0.050*0.8;

% M Moment Coefficients
FD.Aero.Cmo  =  0.2777;         % From Assignment 2
FD.Aero.Cma  = -1.5922;         % Using equation from assignment 2 (C_malpha = a.* (0.235 - No)) and X_cg nominal at 0.235 times the chord length
FD.Aero.Cmq  = -25.2986;        % From assignment 2
FD.Aero.Cmad = -2.210;          % Left the same
FD.Aero.Cmde = -1.822;          % Left the the same due to insig difference in elevator size.

% N Moment Coefficients 
FD.Aero.Cnb  =  0.173;          % scaled see working, similar as vert. tp chars are similar
FD.Aero.Cnbd =  0.0015;         % left the same
FD.Aero.Cnp  = -0.050;
FD.Aero.Cnr  = -0.539;          % was 0.16 review this dont like the way was scaled
FD.Aero.Cnda =  0.0048*0.628;
FD.Aero.Cndr = -0.115*0.8;

% L Moment Coefficients
FD.Aero.Clb  = -0.0852+0.0917-0.0975; % = -0.0910 estimated by minusing the dihedral effect component of Clb (PC9) and adding the PC21 difedral effect
FD.Aero.Clbd = -0.0004;         % Left the same
FD.Aero.Clp  = -0.3105;         % See working
FD.Aero.Clr  =  0.151;          % estimated using typical characteristic values and transforming using change in wing taper ratios
FD.Aero.Clda = -0.164*0.628;    % See explanation for Cnda, for scale explanation 
FD.Aero.Cldr =  0.0302*0.8;     % Ratio for rudder to vertical tail area, scaled through ratio.



%% UPDATE: Change Data for Centre of Gravity
% The general x_cg coefficients.
% The numbers here came from the published solutions to AERO3000 Ass.2
 

    FD.Inertia.x_cg = x_cg;
    FD.Inertia.m = mass;
    
    %Zero_lift angle
%     FD.Aero.alpha_o = -0.0263;

    % Neutral Point
    FD.Aero.No = 0.5110;

    % Longitudinal Stability
%    FD.Aero.CmCL = x_cg - FD.Aero.No;

    % Moments Changing with centre-of-gravity
    FD.Aero.Cma = 4.835*x_cg - 2.4708;
%     FD.Aero.Cmq = (-2/0.9)*(x_cg - 2.61).^2;	% derived from lecture 8
%     FD.Aero.Cmad = -2.210;                      % Left the same
%     FD.Aero.Cmde = -2.43;
%     FD.Aero.Cmo  = 0.2304 - FD.Aero.Cma*FD.Aero.alpha_o;

    % Lift Coefficients Changing with Centre-of-gravity
%     FD.Aero.CLa = FD.Aero.Cma./FD.Aero.CmCL;
%     FD.Aero.CLq = 2*(x_cg - 2.61);          % google: CLq = Cmq/moment_arm
%     %FD.Aero.CLad = ?
%     %FD.Aero.CLde = ?
%     FD.Aero.CLo  = -FD.Aero.CLa*FD.Aero.alpha_o;
    
    % N Moment Coefficients 
    FD.Aero.Cnb  =  0.173*(x_cg - FD.Aero.No)/(0.265 - FD.Aero.No); 

end
