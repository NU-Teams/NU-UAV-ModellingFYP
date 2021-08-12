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
function Rates = AngularRates(X_k, X_dot)
%% INPUTS 

    % Grab values from AeroAngles rates
        [V,~,~] = AeroAngles(X_k);

    % Grab values from the state rates value from the last time step
        v_dot = X_dot(2);
        w_dot = X_dot(3);

%%  FUNCTION 

    % Alpha dot 
        alpha_dot = (w_dot/V); 
    % Beta dot 
        beta_dot = (v_dot/V); 
    
        

%% OUTPUT 
    Rates = [alpha_dot; beta_dot];    

end 