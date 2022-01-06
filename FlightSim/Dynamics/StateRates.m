%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X_dot = StateRates(X_k, U_k, X_dot, FD)
% returns time derivatives of every state.
% NOTE: use quarternions to keep track of altitude 
%
% INPUTS:
%   states vector of the current time-step (X_k)
%   Inputs of the current time-step (U_k)
%   the dynamics of the prior timestep to approximate alpha_dot and
%   beta_dot
%
% CALLED FUNCTIONS:
%	Initialisation(x_cg)
%   DCM(X_k)
%   AeroAngles(X_k)
%   GravForce(X_k)
%   - Initialisation(x_cg)
%       - DCM(X_k)
%   PropForces(X_k, U_k, x_cg)
%        - Initialisation(x_cg)
%        - FlowProperties(X_k)
%           -- AeroAngles(X_k)
%        - AeroAngles(X_k)
%   BodyForces(X_k, U_k, X_dot, x_cg)
%       - Initialisation(x_cg)
%       - DCM(X_k)
%       - FlowProperties(X_k)
%           -- AeroAngles(X_k)
%       - WindForces(X_k,U_k, X_dot, x_cg)
%           -- Initialisation(x_cg)
%           -- AeroAngles(X_k)
%           -- AngularRates(X_k, X_dot)
%               --- AeroAngles(X_k)
%       - AngularRates(X_k, X_dot)
%       - AeroAngles(X_k)
%
% OUTPUTS:
%   13 state rates in one matrix denoted X_dot
%
% Inga Leinasars, 02/05/2021 
% (upated) Inga Leinasars, 03/05/2021
% Jason Iredale 3/5/2021 2152
% Inga Leinasars 07/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_dot] = StateRates(X_k, U_k, X_dot, FD)
%% Call Functions needed for StateRates

% DONE: Ceb is the body --> Earth Transform Matrix
CosineMatrix = DCM(X_k);
Ceb = CosineMatrix.Ceb;

%% initialisation of states & variables from input vectors

    u = X_k(1);
    v = X_k(2);
    w = X_k(3);
    
    velVector = [u; v; w];

%% 3 velocities
    
    vel_dot = velocityRates(X_k, U_k, X_dot, FD);
    
%% 3 rotation rates

    rotVel_dot = rotationalVelocityRates(X_k, U_k, X_dot, FD);
    
%% 4 attitude rates (using quarternions)
    
    quat_dot = quaternionRates(X_k);
    
%% 3 positional components 

    position_e_dot = Ceb*velVector - FD.VW_e;
    
    %{
    x_e_dot = position_dot(1) - FD.VW_e(1); % velocity in the north-heading (LVLH) [m/s]
    y_e_dot = position_dot(2) - FD.VW_e(2); % velocity in the east-heading (LVLH) [m/s]
    z_e_dot = position_dot(3) - FD.VW_e(3); % velocity in the altitude (LVLH) [m/s]
    %}
    
%% STATE RATES VECTOR
    X_dot = [vel_dot;
             rotVel_dot;
             quat_dot;
             position_e_dot];

end

