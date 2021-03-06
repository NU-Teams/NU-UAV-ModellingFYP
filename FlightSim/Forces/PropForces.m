%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Force, Moment] = PropForces(X_k, U_k, FD)
% Returns the forces and moments caused by the propulsion. assumes the
% forces is entirely in the body x-axis and no moments are generated.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Force, Moment, fuel_dot] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT)       


ft = 3.28084;
rps = 1/(2*pi);
rpm = 60*rps;

fuel_dot = 0;

%% Unpack
PropObject  = AIRCRAFT.Prop.object.Text;
P_max       = AIRCRAFT.Prop.P_max;
eta         = AIRCRAFT.Prop.eta;
D           = AIRCRAFT.Prop.diameter;
CT_coeff    = AIRCRAFT.Prop.CT;
CP_coeff    = AIRCRAFT.Prop.CP;

% assume the propulsion thrust acts inline with the body x-axis
% (propulsion axes --> body axes)
C_bp = eye(3);

% Assume the propulsion acts from the centre of gravity 
Dx_cg = 0;
Dy_cg = 0;
Dz_cg = 0;

% Get density
[sigma, ~]	= FlowProperties(X_k, ENVIRONMENT);
rho         = sigma.*ENVIRONMENT.Sealevel.density;

% True airspeed
[VT, ~, ~]	= AeroAngles(X_k);

% Thrust of the engine
delta_T = U_k(1);

%%

if strcmp(PropObject, 'Electric')
    
    % Available battery voltage
    Vtotal	= AIRCRAFT.Prop.Vtotal;
    
    % motor-gyroscope constant
    KV      = AIRCRAFT.Prop.motorKV/rpm;
    % motorgyroscope constants in SI
    K_omega = 1/KV;
    K_I     = K_omega;
    
    % Motor input voltage
    Vin     = Vtotal*delta_T;
    % Motor Speed (SI)
    omega = Vin/K_omega;
    % Motor speed (rev/sec)
    n = omega*rps;
    
    
    
    % propeller
    J = VT./(n*D);
    J_poly = [J.^0; J.^1; J.^2; J.^3]; 
    CT = CT_coeff*J_poly;
    CP = CP_coeff*J_poly;
    CQ = CP/(2*pi);
    eta_p = J*CT/CP;
    
    thrust	= rho*(n.^2)*(D.^4)*CT;
    torque	= rho*(n.^2)*(D.^5)*CQ;
    
    % Thust in the propulsion axes
    thrust	= thrust*[1;
                      0;
                      0];
                        
    % current draw
    I_out   = torque/K_I;
    
    % rate of capacity drop from the battery
    fuel_dot = I_out;
    
    % Battery power output --> motor power input
    P_battery = Vin.*I_out;
    % Motor power output --> propeller power input
    P_brake = torque.*omega;
    % propeller power output
    Power = thrust(1).*VT;
    % Power should equal Power Validate
    Power_validate = P_brake*eta_p;
    
elseif strcmp(PropObject, 'Constant Speed Propeller')
    
    turboAlt    = AIRCRAFT.Prop.turboAlt/ft;
    % Get density of turbo charge
    [turboSigma, ~] = FlowProperties([0;0;0; 0;0;0; 0;0;0;0; 0;0;-turboAlt], ENVIRONMENT);
    
    % turbo charge maintains constant power up to an altitude
    if abs(X_K(13)) < turboAlt
        
        % Brake power of the engine (propellor efficiency NOT included)
        P_brake = P_max;  
        
    else
        
        % Brake power of the engine (propellor efficiency NOT included)
        P_brake = P_max*( sigma/turboSigma + 0.132*(desnityRatio-1) );
        
    end
    
    % Available power of the propeller
    Power	= eta.*P_brake;
    
    thrust	= (Power./VT)*delta_T*[1;
                                   0;
                                   0];
    
elseif strcmp(PropObject, 'Turbo-Prop')
    
    altIndex = AIRCRAFT.Prop.altIndex;
    
    % Power of the engine (propellor efficiency NOT included)
    P_brake	= P_max.*(sigma.^altIndex);
    
    % Available power of the propeller
    Power	= eta.*P_brake;
    
    % Thust in the propulsion axes
    thrust	= (Power./VT)*delta_T*[1;
                                   0;
                                   0];

end

%%

% propulsion forces in the body axes
Force = C_bp*thrust;

% distances from the propulsion device and the centre of gravity
Dr_cg = [Dx_cg;
         Dy_cg;
         Dz_cg];

% Moments generated from the propulsion
Moment = cross(Dr_cg, Force);

end