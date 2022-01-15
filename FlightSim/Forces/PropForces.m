%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Force, Moment] = PropForces(X_k, U_k, FD)
% Returns the forces and moments caused by the propulsion. assumes the
% forces is entirely in the body x-axis and no moments are generated.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Force, Moment] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT)       



%% Unpack
P_max	= AIRCRAFT.Prop.P_max;
eta     = AIRCRAFT.Prop.eta;

% assume the propulsion axes is inline with the body x-axis
C_bp = eye(3);

% Assume the propulsion acts from the centre of gravity 
Dx_cg = 0;
Dy_cg = 0;
Dz_cg = 0;

% Get density ratio and true-airspeed
[densityRatio, ~] = FlowProperties(X_k, ENVIRONMENT);
[VT, ~, ~] = AeroAngles(X_k);
    
% Power of the engine (propellor efficiency NOT included)
Power = P_max.*(densityRatio.^0.7);

% Thrust of the engine (propellor efficiency included)
delta_T = U_k(1);

% Thust in the propulsion axes
thrust = eta.*(Power./VT)*delta_T*[1;
                                   0;
                                   0];
                               
% propulsion forces in the body axes
Force = C_bp*thrust;


% distances from the propulsion device and the centre of gravity
Dr = [Dx_cg;
     Dy_cg;
     Dz_cg];


% Moments generated from the propulsion
Moment = cross(Dr, Force);

end