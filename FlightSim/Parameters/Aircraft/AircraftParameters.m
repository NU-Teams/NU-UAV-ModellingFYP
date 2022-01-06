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

function AircraftSpec = AircraftParameters(x_cg, mass)

% Note: FD --> FlightData

%% Inertial Data

% --> Inertia_PC21 = Mratio*OtherRatio*Inertia_PC9

AircraftSpec.Inertia.g = 9.81;                                % Gravity Constant
AircraftSpec.Inertia.m = 2650;                                % Aircraft Mass (kg)
AircraftSpec.Inertia.Ixx = 5066*(2650/2087)*(9.11/10.19)^1;   % Aircraft Moments of Inertia (kg.m^2)
AircraftSpec.Inertia.Iyy = 6578*(2650/2087)*(11.23/10.18)^1;  % Aircraft Moments of Inertia (kg.m^2)
AircraftSpec.Inertia.Izz = 10975*(2650/2087)*(3.75/3.26)^1;   % Aircraft Moments of Inertia (kg.m^2)
AircraftSpec.Inertia.Ixz = 203*(2650/2087);                   % Aircraft Product of Inertia (kg.m^2)

%% Geometric Data
AircraftSpec.Geom.S = 15.22;              % Platform Area (m^2)
AircraftSpec.Geom.c = 2.32;               % Chord Length (m)
AircraftSpec.Geom.b = 9.11;               % Wing Span (m)

%% Propulsion Data
% Source:
% <http://www.mathworks.com https://en.wikipedia.org/wiki/Pratt_%26_Whitney_Canada_PT6#Specifications_(PT6A-6)>

AircraftSpec.Prop.P_max = 1200000;        % Maximum engine power (Watts)
AircraftSpec.Prop.eta = 0.8;              % Propeller efficiency

%% Control Data
DtoR = pi/180;

AircraftSpec.CtrlLim.Lwr = [0;            % Throttle range (Fraction)
                  -25*DtoR;     % Elevator range (rad)
                  -25*DtoR;     % Aileron range (rad)
                  -25*DtoR];	% Rudder range (rad)
AircraftSpec.CtrlLim.Upr = [1;            % Throttle range (Fraction)
                  25*DtoR;      % Elevator range (rad)
                  25*DtoR;      % Aileron range (rad)
                  25*DtoR];     % Rudder range (rad)

%% Aerodynamic Data 

% Zero-Lift Angle
AircraftSpec.Aero.alpha_o = -0.0249;      % Calculated in Assignment 2 -0.0249 rads

% Drag Coefficients
AircraftSpec.Aero.Cdo    =  0.023;        % CD = CD0 + k*CL^2, CD0 = 0.023
AircraftSpec.Aero.k      =  0.0663;       % k = 1/(pi*AR*e)

% As long as you can justify it its fine!!!
% Lift Coefficients
AircraftSpec.Aero.CLa  =  4.871;          % a as calculated in assignment 2
AircraftSpec.Aero.CLq  =  9.771;          % CL/turn rate deriv of pitch rate in terms of lift
AircraftSpec.Aero.CLad = -1.987;          % Left the same
AircraftSpec.Aero.CLde =  0.532;          % elevator deflection very similar thus unchanged.
AircraftSpec.Aero.CLo  = -AircraftSpec.Aero.CLa*AircraftSpec.Aero.alpha_o; %offset


% Side Force Coefficients
AircraftSpec.Aero.Cyb  = -0.712;
AircraftSpec.Aero.Cybd = -0.0032;         % Left the same
AircraftSpec.Aero.Cyp  = -0.205;
AircraftSpec.Aero.Cyr  =  0.542;
AircraftSpec.Aero.Cyda =  0.000;
AircraftSpec.Aero.Cydr =  0.050*0.8;

% M Moment Coefficients
AircraftSpec.Aero.Cmo  =  0.2777;         % From Assignment 2
AircraftSpec.Aero.Cma  = -1.5922;         % Using equation from assignment 2 (C_malpha = a.* (0.235 - No)) and X_cg nominal at 0.235 times the chord length
AircraftSpec.Aero.Cmq  = -25.2986;        % From assignment 2
AircraftSpec.Aero.Cmad = -2.210;          % Left the same
AircraftSpec.Aero.Cmde = -1.822;          % Left the the same due to insig difference in elevator size.

% N Moment Coefficients 
AircraftSpec.Aero.Cnb  =  0.173;          % scaled see working, similar as vert. tp chars are similar
AircraftSpec.Aero.Cnbd =  0.0015;         % left the same
AircraftSpec.Aero.Cnp  = -0.050;
AircraftSpec.Aero.Cnr  = -0.539;          % was 0.16 review this dont like the way was scaled
AircraftSpec.Aero.Cnda =  0.0048*0.628;
AircraftSpec.Aero.Cndr = -0.115*0.8;

% L Moment Coefficients
AircraftSpec.Aero.Clb  = -0.0852+0.0917-0.0975; % = -0.0910 estimated by minusing the dihedral effect component of Clb (PC9) and adding the PC21 difedral effect
AircraftSpec.Aero.Clbd = -0.0004;         % Left the same
AircraftSpec.Aero.Clp  = -0.3105;         % See working
AircraftSpec.Aero.Clr  =  0.151;          % estimated using typical characteristic values and transforming using change in wing taper ratios
AircraftSpec.Aero.Clda = -0.164*0.628;    % See explanation for Cnda, for scale explanation 
AircraftSpec.Aero.Cldr =  0.0302*0.8;     % Ratio for rudder to vertical tail area, scaled through ratio.



%% Change Data for Centre of Gravity
% The numbers here came from the published solutions to AERO3000 Ass.2

% TODO: Find how the non-dimensional coefficients change w.r.t. change in a
% centre of gravity.

    AircraftSpec.Inertia.x_cg = x_cg;
    AircraftSpec.Inertia.m = mass;
    
    %Zero_lift angle
%     FD.Aero.alpha_o = -0.0263;

    % Neutral Point
    AircraftSpec.Aero.No = 0.5110;

    % Longitudinal Stability
%    FD.Aero.CmCL = x_cg - FD.Aero.No;

    % Moments Changing with centre-of-gravity
    AircraftSpec.Aero.Cma = 4.835*x_cg - 2.4708;
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
    AircraftSpec.Aero.Cnb  =  0.173*(x_cg - AircraftSpec.Aero.No)/(0.265 - AircraftSpec.Aero.No); 

    
    
    
end
