function [omega_dot] = rotationalVelocityRates(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT)

%% UNPACK

Ixx = AIRCRAFT.Inertia.Ixx;
Iyy = AIRCRAFT.Inertia.Iyy;
Izz = AIRCRAFT.Inertia.Izz;
Ixy = AIRCRAFT.Inertia.Ixy;
Iyz = AIRCRAFT.Inertia.Iyz;
Ixz = AIRCRAFT.Inertia.Ixz;

% Moments generated from the aerodynamics
[~, aero_M] = BodyForces(X_k, U_k, X_dot, AIRCRAFT, ENVIRONMENT);

% Moments generated from the propulsion
[~, propulsive_M, ~] = PropForces(X_k, U_k, AIRCRAFT, ENVIRONMENT);


%% initialisation of states & variables from input vectors

    % unpack rotational velocities
    p       = X_k(4);
    q       = X_k(5);
    r       = X_k(6);
    
    omega   = [p;
               q;
               r];
    
    Moment = aero_M + propulsive_M;
    
    % pack Inertias into an Inertia matrix
    I = [-Ixx  Ixy  Ixz;
          Ixy -Iyy  Iyz;
          Ixz  Iyz -Izz];
    
    
%% 3 rotation rates

    omega_dot = (I^-1)*(cross(I*omega, omega) - Moment);

end