%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CL, CD] = WindForces(X_k, U_k, X_dot, FD)
% Returns the lift and drag coefficient.
% This requires
%
% CALLED FUNCTIONS:
%   AeroAngles(X_k)
%   AngularRates(X_k, X_dot)

% Jason Iredale, 30/04/2021
% Jason Iredale, 07,05,2021 0150
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [CL, CD] = WindForces(X_k, U_k, X_dot, FD)
%% CODE

% Grab values from AeroAngles rates
[VT,alpha,~] = AeroAngles(X_k);

rates = AngularRates(X_k, X_dot);
alpha_dot = rates(1);

% Elevator Deflection
delta_e = U_k(2,:);

% Aerodynamic Pitch-rate
q = X_k(5,:);
q_hat = q*FD.Geom.c./(2*VT);

% Lift Coefficient
CL = FD.Aero.CLo + FD.Aero.CLa*alpha + FD.Aero.CLde*delta_e + FD.Aero.CLad*alpha_dot + FD.Aero.CLq*q_hat;

% Drag Coefficient
CD = FD.Aero.Cdo + FD.Aero.k*CL.^2;

end