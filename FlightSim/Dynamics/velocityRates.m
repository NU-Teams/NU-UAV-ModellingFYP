function vel_dot = velocityRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT)

%% Unpack
m = AIRCRAFT.Inertia.m;

% force due to gravity
Grav_F = GravForce(X_k, ENVIRONMENT)*m;
    
% Aerodynamic forces and moments
[Aero_F, ~] = BodyForces(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT);

% forces and moments generated from the propulsive device
[Prop_F, ~, ~] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT);


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
    

%% Dynamic Equation for velocity

vel_dot = cross(velocity, angularVel) + Forces/m;
    

end