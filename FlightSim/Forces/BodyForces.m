%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Body_Forces,Body_Moments] = BodyForces(X_k, U_k, X_dot, FD)
%
% Returns the forces [Fx Fy Fz] and moments [Ml Mm Mm] in terms of the body
% axis for a given state and input.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Body_Forces, Body_Moments] = BodyForces(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT)

%% UNPACK

S = AIRCRAFT.Geom.S;
c = AIRCRAFT.Geom.c;
b = AIRCRAFT.Geom.b;


%% FUNCTION

% Get dynamic pressure
[~,Q] = FlowProperties(X_k, ENVIRONMENT);

% Coefficients
C = WindForces(X_k,U_k, X_dot, AIRCRAFT);

CD = C(1);
CY = C(2);
CL = C(3);
Cl = C(4);
Cm = C(5);
Cn = C(6);

% alpha & beta
[~, alpha, ~] = AeroAngles(X_k);

% Staility axis
C_bs     = stability2body(alpha);


% Calculate the aerodynamic forces in the stability frame
    % Lift 
        F_L = CL*Q*S; 
    % Drag
        F_D = CD*Q*S; 
    % Sideforce 
        F_Y = CY*Q*S;
        
% Calculate the aerodynamic moments
    % Roll Moment L
        M_l = Cl*Q*S*b; 
    % Pitch Moment M
        M_m = Cm*Q*S*c; 
    % Yaw Moment N
        M_n = Cn*Q*S*b;  


%% OUTPUTS

% Forces, note drag and lift point in the opposite direction to +ve x and z
Stability_Forces = [-F_D;
                     F_Y;
                    -F_L];
% rotates from stability axes to body axes
Body_Forces = C_bs*Stability_Forces;

% Moments
    Body_Moments = [M_l;
                    M_m;
                    M_n];


end 