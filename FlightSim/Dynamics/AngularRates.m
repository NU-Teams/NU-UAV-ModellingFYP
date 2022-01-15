%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rates = AngularRates(X_k, X_dot)
% Finds alpha_dot and beta_dot for a given state (X_k) and dynamics of a
% state (X_dot). In AERO3000 FlightSim we will just use the dynamics from
% the previous timestep.
%
% CALLED FUNCTIONS:
%   AeroAngles(X_k)
%  Asleigh Rattray
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [V_dot, beta_dot, alpha_dot] = AngularRates(X_k, X_dot)
%% INPUTS 
    
    % unpack
        u               = X_k(1);
        v               = X_k(2);
        w               = X_k(3);

    % Grab values from AeroAngles rates
        [V    , ~, ~]   = AeroAngles(X_k);
        [V_dot, ~, ~]   = AeroAngles(X_dot);

    % Grab values from the state rates value from the last time step
        u_dot           = X_dot(1);
        v_dot           = X_dot(2);
        w_dot           = X_dot(3);

%%  FUNCTION 

    % Alpha dot = d/dt atan(f(x)/g(x))
        alpha_dot = (w_dot*u - u_dot*w) / (u^2 - w^2); 
    % Beta dot  = d/dt asin(f(x)/g(x))
        beta_dot = (v_dot - V_dot*(v/V)) / sqrt(V^2 - v^2); 

end 