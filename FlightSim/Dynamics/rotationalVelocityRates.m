function [rotVel_dot] = rotationalVelocityRates(X_k, U_k, X_dot, FD)
    
% DONE: aero_F & aero_M
[~, aero_M] = BodyForces(X_k, U_k, X_dot, FD);

% DONE: propulsive_F & propulsive_M
[~, propulsive_M] = PropForces(X_k, U_k, FD);


%% initialisation of states & variables from input vectors

    p = X_k(4);
    q = X_k(5);
    r = X_k(6);
    
    MomentVector = aero_M + propulsive_M;
    
    
%% inertial constants

    % p_dot and q_dot
    C0 = FD.Inertia.Ixx*FD.Inertia.Izz - FD.Inertia.Ixz^2;
    C1 = FD.Inertia.Izz/C0;
    C2 = FD.Inertia.Ixz/C0;
    C3 = C2*(FD.Inertia.Ixx - FD.Inertia.Iyy + FD.Inertia.Izz);
    C4 = C1*(FD.Inertia.Iyy - FD.Inertia.Izz) - C2*FD.Inertia.Ixz;
    C8 = FD.Inertia.Ixx/C0;
    C9 = C8*(FD.Inertia.Ixx - FD.Inertia.Iyy) + C2*FD.Inertia.Ixz;
    
    % q_dot
    C5 = 1/FD.Inertia.Iyy;
    C6 = C5*FD.Inertia.Ixz;
    C7 = C5*(FD.Inertia.Izz - FD.Inertia.Ixx);

    
%% 3 rotation rates

    %{
    p_dot = C3*p*q + C4*q*r       + C1*(L_A+L_T) + C2*(N_A+N_T);	% change in roll-rate [rads/s^2]
    q_dot = C7*p*r - C6*(p^2-r^2) + C5*(M_A+M_T);                   % change in pitch-rate [rads/s^2]
    r_dot = C9*p*q - C3*q*r       + C2*(L_A+L_T) + C8*(N_A+N_T);    % change is yaw-rate [rads/s^2]
    %}
    
    coupledpqrMatrix = [C3 0  C4  0 ;
                        0  C7 0  -C6;
                        C9 0 -C3  0 ];
    coupledpqrVector = [p*q; p*r; q*r; p^2-r^2];
   
    MomentMatrix = [C1 0  C2;
                    0  C5 0 ;
                    C2 0  C8];
    
    rotVel_dot = coupledpqrMatrix*coupledpqrVector + MomentMatrix*MomentVector;


end