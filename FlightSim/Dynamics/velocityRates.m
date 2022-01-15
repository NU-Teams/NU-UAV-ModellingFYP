function vel_dot = velocityRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT)

%% Unpack
m = AIRCRAFT.Inertia.m;

% DONE: get gravity in terms of body ref. frame
Grav_F = GravForce(X_k, ENVIRONMENT)*m;
    
% DONE: aero_F & aero_M
[Aero_F, ~] = BodyForces(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT);

% DONE: propulsive_F & propulsive_M
[Prop_F, ~] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT);


%% initialisation of states & variables from input vectors

u = X_k(1);
v = X_k(2);
w = X_k(3);
p = X_k(4);
q = X_k(5);
r = X_k(6);

velocity  = [u;
             v;
             w];

angularVel = [p;
             q;
             r];

Forces = Grav_F + Aero_F + Prop_F;
    

%% 3 velocities
    
    %{
    u_dot =  r*v - q*w + g_x + (F_A_x + F_T_x)/m;    % velocity in longitudinal axis (BODY) [m/s]
    v_dot = -r*u + p*w + g_y + (F_A_y + F_T_y)/m;    % velocity in lateral axis (BODY) [m/s]
    w_dot =  q*u - p*v + g_z + (F_A_z + F_T_z)/m;    % velocity in normal (BODY) [m/s]
    %}
        
    vel_dot = cross(velocity, angularVel) + Forces/m;
    

end