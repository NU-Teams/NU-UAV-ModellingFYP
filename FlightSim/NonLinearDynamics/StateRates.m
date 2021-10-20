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

% DONE: get gravity in terms of body ref. frame
GF_m = GravForce(X_k, FD);
    
% DONE: aero_F & aero_M
[aero_F, aero_M] = BodyForces(X_k, U_k, X_dot, FD);

% DONE: propulsive_F & propulsive_M
[propulsive_F, propulsive_M] = PropForces(X_k, U_k, FD);

% DONE: m
m = FD.Inertia.m;


%% initialisation of states & variables from input vectors

    u = X_k(1);
    v = X_k(2);
    w = X_k(3);
    p = X_k(4);
    q = X_k(5);
    r = X_k(6);
    q0 = X_k(7);
    q1 = X_k(8);
    q2 = X_k(9); 
    q3 = X_k(10);
    x_e = X_k(11);
    y_e = X_k(12);
    z_e = X_k(13);
    
    velVector = [u; v; w];
    pqrVector = [p; q; r];
    quatVector = [q0; q1; q2; q3];
    posVector = [x_e; y_e; z_e];
    
    Forces_mVector = GF_m + (aero_F + propulsive_F)/m;
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
    
    pqr_dot = coupledpqrMatrix*coupledpqrVector + MomentMatrix*MomentVector;
    
%% 4 attitude rates (using quarternions)
    
    %{
    q0_dot = -(1/2)*(q1*p + q2*q + q3*r);
    q1_dot =  (1/2)*(q0*p - q3*q + q2*r);
    q2_dot =  (1/2)*(q3*p + q0*q - q1*r);
    q3_dot = -(1/2)*(q2*p - q1*q - q0*r);
    %}
    
    quat_pqrMatrix = (1/2)*[0 -p -q -r;
                            p  0  r -q;
                            q -r  0  p;
                            r  q -p  0];
    
    quat_dot = quat_pqrMatrix*quatVector;
    
%% 3 positional components 

    position_e_dot = Ceb*velVector - FD.VW_e;
    
    %{
    x_e_dot = position_dot(1) - FD.VW_e(1); % velocity in the north-heading (LVLH) [m/s]
    y_e_dot = position_dot(2) - FD.VW_e(2); % velocity in the east-heading (LVLH) [m/s]
    z_e_dot = position_dot(3) - FD.VW_e(3); % velocity in the altitude (LVLH) [m/s]
    %}
    
%% STATE RATES VECTOR
    X_dot = [vel_dot;
             pqr_dot;
             quat_dot;
             position_e_dot];

end

