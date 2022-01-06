function vel_dot = velocityRates(X_k, U_k, X_dot, FD)


% DONE: get gravity in terms of body ref. frame
GF_m = GravForce(X_k, FD);
    
% DONE: aero_F & aero_M
[aero_F, ~] = BodyForces(X_k, U_k, X_dot, FD);

% DONE: propulsive_F & propulsive_M
[propulsive_F, ~] = PropForces(X_k, U_k, FD);

% DONE: m
m = FD.Inertia.m;


%% initialisation of states & variables from input vectors

    u = X_k(1);
    v = X_k(2);
    w = X_k(3);
    p = X_k(4);
    q = X_k(5);
    r = X_k(6);

    
    velVector = [u; v; w];
    
    Forces_mVector = GF_m + (aero_F + propulsive_F)/m;
    
    


%% 3 velocities
    
    %{
    u_dot =  r*v - q*w + g_x + (F_A_x + F_T_x)/m;    % velocity in longitudinal axis (BODY) [m/s]
    v_dot = -r*u + p*w + g_y + (F_A_y + F_T_y)/m;    % velocity in lateral axis (BODY) [m/s]
    w_dot =  q*u - p*v + g_z + (F_A_z + F_T_z)/m;    % velocity in normal (BODY) [m/s]
    %}
    
    vel_pqrMatrix = [ 0  r -q;
                     -r  0  p;
                      q -p  0];
                  
    vel_dot = vel_pqrMatrix*velVector + Forces_mVector;
    

end