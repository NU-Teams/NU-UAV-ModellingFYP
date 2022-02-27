%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X_dot = StateRates(X_k, U_k, X_dot, FD)
% returns time derivatives of every state.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_dot] = StateRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT)


%% 3 velocities
    
    vel_dot = velocityRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT);
    
%% 3 rotation rates

    rotVel_dot = rotationalVelocityRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT);
    
%% 4 attitude rates (using quarternions)
    
    quat_dot = quaternionRates(X_k);
    
%% 3 positional components 

    position_e_dot = positionRates(X_k, ENVIRONMENT);
    
%% fuel

    [~, ~, fuel_dot] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT);
    % todo: wrap fuel consumption into this
    
%% STATE RATES VECTOR
    X_dot = [vel_dot;
             rotVel_dot;
             quat_dot;
             position_e_dot];

end

