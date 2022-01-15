%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [C] = WindForces(X_k, U_k, X_dot, FD)
% Returns the lift and drag coefficient.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C] = WindForces(X_k, U_k, X_dot, AIRCRAFT)

% unpack
c = AIRCRAFT.Geom.c;
b = AIRCRAFT.Geom.b;

dragCoefficients        = [AIRCRAFT.Aero.CDo AIRCRAFT.Aero.k];
liftCoefficients        = [AIRCRAFT.Aero.CLo AIRCRAFT.Aero.CLa AIRCRAFT.Aero.CLad AIRCRAFT.Aero.CLq AIRCRAFT.Aero.CLde];
pitchmomentCoefficients	= [AIRCRAFT.Aero.Cmo AIRCRAFT.Aero.Cma AIRCRAFT.Aero.Cmad AIRCRAFT.Aero.Cmq AIRCRAFT.Aero.Cmde];

sideforceCoefficients	= [AIRCRAFT.Aero.CYo AIRCRAFT.Aero.CYb AIRCRAFT.Aero.CYbd AIRCRAFT.Aero.CYp AIRCRAFT.Aero.CYr AIRCRAFT.Aero.CYda AIRCRAFT.Aero.CYdr];
rollmomentCoefficients	= [AIRCRAFT.Aero.Clo AIRCRAFT.Aero.Clb AIRCRAFT.Aero.Clbd AIRCRAFT.Aero.Clp AIRCRAFT.Aero.Clr AIRCRAFT.Aero.Clda AIRCRAFT.Aero.Cldr];
yawmomentCoefficients	= [AIRCRAFT.Aero.Cno AIRCRAFT.Aero.Cnb AIRCRAFT.Aero.Cnbd AIRCRAFT.Aero.Cnp AIRCRAFT.Aero.Cnr AIRCRAFT.Aero.Cnda AIRCRAFT.Aero.Cndr];


%% CODE

% Grab values from AeroAngles rates
[VT, beta, alpha] = AeroAngles(X_k);

[~, beta_dot, alpha_dot] = AngularRates(X_k, X_dot);

% non-dimensionalised angle rates
alpha_dot_hat	= alpha_dot*(c/(2*VT)); 
beta_dot_hat	= beta_dot *(b/(2*VT)); 

p = X_k(4) ; % Roll Rate 
q = X_k(5) ; % Pitch Rate
r = X_k(6) ; % Yaw Rate

% non-dimensionalised angle rates
p_hat = p*b./(2*VT);
q_hat = q*c./(2*VT);
r_hat = r*b./(2*VT);

% Grab Values from control matrix Control =
% [delta_T,delta_e,delta_a,delta_r]
delta_e = U_k(2); % Elevator Deflection 
delta_a = U_k(3); % Aileron Deflection 
delta_r = U_k(4); % Rudder Deflection

longVariables = [1; alpha; alpha_dot_hat; q_hat; delta_e];
latVariables  = [1; beta; beta_dot_hat; p_hat; r_hat; delta_a; delta_r];

% Lift Coefficient
CL = liftCoefficients*longVariables;

dragPolar = [1; CL^2];
% Drag Coefficient
CD = dragCoefficients*dragPolar;

% Pitch Moment Coefficient 
Cm = pitchmomentCoefficients*longVariables;

%  Roll Moment Coefficient
Cl = rollmomentCoefficients*latVariables;

% Sideforce Coefficient 
CY = sideforceCoefficients*latVariables;

%  Yaw Moment Coefficient 
Cn = yawmomentCoefficients*latVariables;

C = [CD CY CL Cl Cm Cn];
        
end