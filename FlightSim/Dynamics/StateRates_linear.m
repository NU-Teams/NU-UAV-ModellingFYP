%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X_dot = StateRates(X_k, U_k, X_dot, FD)
% returns time derivatives of every state.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_dot_12] = StateRates_linear(X_k_12, U_k, X_dot, AIRCRAFT, ENVIRONMENT)

CHI  = X_k_12(7:9);
quat = E2Q(CHI);

X_k_13 = [X_k_12(1:3);
          X_k_12(4:6);
          quat;
          X_k_12(10:12)];
   
%% 3 velocities
    
    vel_dot = velocityRates(X_k_13, U_k, X_dot, AIRCRAFT, ENVIRONMENT);
    
%% 3 rotation rates

    rotVel_dot = rotationalVelocityRates(X_k_13, U_k, X_dot, AIRCRAFT, ENVIRONMENT);
    
%% 4 attitude rates (using quarternions)
    
    eul_dot = attitudeRates(X_k_12);
    
%% 3 positional components 

    position_e_dot = positionRates(X_k_13, ENVIRONMENT);
    
%% STATE RATES VECTOR
    X_dot_12 = [vel_dot;
                rotVel_dot;
                eul_dot;
                position_e_dot];

end
