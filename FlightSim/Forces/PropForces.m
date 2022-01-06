%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Force, Moment] = PropForces(X_k, U_k, FD)
% Returns the forces and moments caused by the propulsion. assumes the
% forces is entirely in the body x-axis and no moments are generated.
%
% CALLED FUNCTIONS:
%   Initialisation()
%   FlowProperties(X_k)
%       - AeroAngles(X_k)
%   AeroAngles(X_k)
%   
%
% Jason Iredale, 30/04/2021
% Inga Leinasars, 02/05/2021
% Jason Iredale 3/5/2021 2206
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Force, Moment] = PropForces(X_k, U_k, FD)       
    %% --------- INGA: determine delta_T & TAS----------- %%         
% OUTPUTS:
    % Power             % 
    % Thrust            %
    
%% CODE

% Get density ratio and true-airspeed
[density_ratio, ~] = FlowProperties(X_k);
[true_airspeed, ~, ~] = AeroAngles(X_k);
    
% Power of the engine (propellor efficiency NOT included)
Power = FD.Prop.P_max.*(density_ratio.^0.7);

% Thrust of the engine (propellor efficiency included)
delta_T = U_k(1);
F_x = FD.Prop.eta.*Power*(1/true_airspeed)*delta_T;
F_y = 0;     % no thrust in the lateral axis
F_z = 0;     % no thrust in the normal axis

Force = [F_x; F_y; F_z];

% Assuming the thrust is inline with the centre of gravity
M_l = 0;
M_m = 0;
M_n = 0;
Moment = [M_l; M_m; M_n];

end