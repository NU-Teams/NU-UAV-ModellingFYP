%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function generates and returns the flight data structure which
% defines the aircraft data and aerodynamic derivatives.
%
% Aircraft data and aerodynamic derivatives  - CG 22% mac.
% Flight config = clean_xx - flaps deflected 0 deg, clean configuration
% 
% (c) Peter W. Gibbens & S. Dumble, 1 March, 2011.
% (Updated) Peter W. Gibbens, 29 April, 2021.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ FD ] = PC9_Flight_Data_2021()

    % Note: FD --> FlightData
    
    % Inertial Data
    FD.Inertia.g = 9.81;           % Gravity Constant
    FD.Inertia.m = 2087;           % Aircraft Mass (kg)
    FD.Inertia.Ixx = 5066;         % Aircraft Moments of Inertia (kg.m^2)
    FD.Inertia.Iyy = 6578;         % Aircraft Moments of Inertia (kg.m^2)
    FD.Inertia.Izz = 10975;        % Aircraft Moments of Inertia (kg.m^2)
    FD.Inertia.Ixz = 203;          % Aircraft Moments of Inertia (kg.m^2)
    
    % Geometric Data
    FD.Geom.S = 16.29;         % Platform Area (m^2)
    FD.Geom.c = 1.652;         % Chord Length (m)
    FD.Geom.b = 10.12;         % Wing Span (m)
    
    % Propulsion Data
    FD.Prop.P_max = 708000; % Maximum engine power (Watts)
    FD.Prop.eta = 0.8; % Propeller efficiency
    
    % Control Data
    DtoR = pi/180;
    
    FD.CtrlLim.Lwr = [0;            % Throttle range (Fraction)
                                      -25*DtoR;     % Elevator range (rad)
                                      -25*DtoR;     % Aileron range (rad)
                                      -25*DtoR];    % Rudder range (rad)
    FD.CtrlLim.Upr = [1;            % Throttle range (Fraction)
                                      25*DtoR;      % Elevator range (rad)
                                      25*DtoR;      % Aileron range (rad)
                                      25*DtoR];     % Rudder range (rad)

    % Aerodynamic Data 
    FD.Aero.alpha_o = -3.0/57.3; 
    % Drag Coefficients
    FD.Aero.Cdo    =  0.0215;
    FD.Aero.k      =  0.0555;
    % Lift Coefficients
    FD.Aero.CLa  =  4.659;
    FD.Aero.CLq  =  7.960;
    FD.Aero.CLad = -1.987;
    FD.Aero.CLde =  0.532;
    FD.Aero.CLo  = -FD.Aero.CLa*FD.Aero.alpha_o;
    % Side Force Coefficients
    FD.Aero.Cyb  = -0.507;
    FD.Aero.Cybd = -0.0032;
    FD.Aero.Cyp  = -0.128;
    FD.Aero.Cyr  =  0.336;
    FD.Aero.Cyda =  0.000;
    FD.Aero.Cydr =  0.050;
    % M Moment Coefficients
    FD.Aero.Cmo  =  0.06;
    FD.Aero.Cma  = -0.802;
    FD.Aero.Cmq  = -17.72;
    FD.Aero.Cmad = -2.210;
    FD.Aero.Cmde = -1.822;
    % N Moment Coefficients
    FD.Aero.Cnb  =  0.107;
    FD.Aero.Cnbd =  0.0015;
    FD.Aero.Cnp  = -0.0226;
    FD.Aero.Cnr  = -0.160;
    FD.Aero.Cnda =  0.0048;
    FD.Aero.Cndr = -0.115;
    % L Moment Coefficients
    FD.Aero.Clb  = -0.0852;
    FD.Aero.Clbd = -0.0004;
    FD.Aero.Clp  = -0.328;
    FD.Aero.Clr  =  0.0776;
    FD.Aero.Clda = -0.164; 
    FD.Aero.Cldr =  0.0302;

end      
