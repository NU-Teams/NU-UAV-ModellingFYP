%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Body_Forces,Body_Moments] = BodyForces(X_k, U_k, X_dot, FD)
%
% Returns the forces [Fx Fy Fz] and moments [Ml Mm Mm] in terms of the body
% axis for a given state and input.
% This also requires alpha_dot and beta_dot so the dynamics (X_dot) is
% needed.
%
% CALLED FUNCTIONS:
%   DCM(X_k)
%   FlowProperties(X_k)
%       - AeroAngles(X_k)
%   WindForces(X_k,U_k, X_dot)
%       - Initialisation(x_cg)
%       - AeroAngles(X_k)
%       - AngularRates(X_k, X_dot)
%           -- AeroAngles(X_k)
%   AngularRates(X_k, X_dot)
%       - AeroAngles(X_k)
%   AeroAngles(X_k)
%   
%
% Ashleigh Rattray, 04/05/2021
% Jason Iredale, 07/05/2021 0205
% Jason Iredale, 12/05/2021 1640
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Body_Forces,Body_Moments] = BodyForces(X_k, U_k, X_dot, FD)
%% INPUTS 


% Get dynamic pressure
[~,Q] = FlowProperties(X_k);

% CL & CD
[CL, CD] = WindForces(X_k,U_k, X_dot, FD);

% alpha & beta
[V,alpha,beta] = AeroAngles(X_k);

% Staility axis
C_alpha =C_y(alpha);
Cbs = C_alpha;

% alpha_dot % beta_dot
Aero_Rates = AngularRates(X_k, X_dot);
alpha_dot = Aero_Rates(1); % similar to pitch rate
beta_dot = Aero_Rates(2); % Yaw Rate

%non-dimensionalised inputs
    % alpha dot hat
        alpha_dot_hat = (alpha_dot*FD.Geom.c)/(2*V); 
    % beta dot hat
        beta_dot_hat = (beta_dot*FD.Geom.c)/(2*V); 


p = X_k(4) ; % Roll Rate 
q = X_k(5) ; % Pitch Rate
r = X_k(6) ; % Yaw Rate 

% Grab Values from control matrix Control =
% [delta_T,delta_e,delta_a,delta_r]
delta_e = U_k(2); % Elevator Deflection 
delta_a = U_k(3); % Aileron Deflection 
delta_r = U_k(4); % Rudder Deflection


%% FUNCTION 

% Calculate the aerodynamic moments vector [L_A,M_A,N_A]'

% Calculate the relative coefficients (Cl (Roll Moment
% Coefficient), Cm (Pitch moment Coefficient), Cn (Yaw Moment))
% --> Assuming that the aircraft is symmetrical and Cl0 = Cn0 = 0

% Roll Moment Coefficient Cl
    % Components needed to calculate coefficient 
        % Aerodynamic Roll Rate 
            p_bar = (p*FD.Geom.b)/(2*V); 
        % Aerodynamic Yaw Rate 
            r_bar = (r*FD.Geom.b)/(2*V);    
    % Coefficient
        Cl = FD.Aero.Clb*beta + FD.Aero.Clp*p_bar + FD.Aero.Clr*r_bar + FD.Aero.Clbd*beta_dot_hat + FD.Aero.Clda*delta_a + FD.Aero.Cldr*delta_r; 

% Yaw Moment Coefficient Cn
    % Coefficient 
        Cn = FD.Aero.Cnb*beta + FD.Aero.Cnp*p_bar + FD.Aero.Cnr*r_bar + FD.Aero.Cnbd*beta_dot_hat + FD.Aero.Cnda*delta_a + FD.Aero.Cndr*delta_r; 

% Pitch Moment Coefficient Cm
    % Components needed to calculate coefficient 
        q_bar = (q*FD.Geom.c)/(2*V); 
    % Coefficient 
        Cm = FD.Aero.Cmo + FD.Aero.Cma*alpha + FD.Aero.Cmq*q_bar + FD.Aero.Cmad*alpha_dot_hat + FD.Aero.Cmde*delta_e; 

% Calculating the Moments 
    % Roll Moment L
        M_L = Cl*Q*FD.Geom.S*FD.Geom.b; 
    % Pitch Moment M
        M_M = Cm*Q*FD.Geom.S*FD.Geom.c; 
    % Yaw Moment N
        M_N = Cn*Q*FD.Geom.S*FD.Geom.b;  

% Calculate the aerodynamic forces vector [F_A_x,F_A_y,F_A_z]' or
% [Drag(D) Sideforce(Y) Lift(L)]; 

% Compute Lift and Drag from Coefficients
    % Lift 
        F_L = CL*Q*FD.Geom.S; 
    % Drag
        F_D = CD*Q*FD.Geom.S; 

% Find the sideforce
    % Sideforce Coefficient 
        CY = FD.Aero.Cyb*beta + FD.Aero.Cybd*beta_dot + FD.Aero.Cyp*p_bar + FD.Aero.Cyr*r_bar + FD.Aero.Cyda*delta_a + FD.Aero.Cydr*delta_r; 
    % Sideforce 
        F_Y = CY*Q*FD.Geom.S; 


%% OUTPUTS

    % Moments
    % Do moments care about reference frames?
        Body_Moments = [M_L,M_M,M_N]';
        Body_Moments = [M_L,M_M,M_N]';
    % Forces
        % Matrix
            Force_Matrix = [-F_D F_Y -F_L]';
        % Body Transformation
        % TODO: turn this into a transform from the stability axis to the
        % body axis
            Body_Forces = Cbs*Force_Matrix;


end 