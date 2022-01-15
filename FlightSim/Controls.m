%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [U, U_impulse, question_str] = Controls(X_in, U_in, T, FD)
% Inputs
%   X_k is a state vector at a certain time step.
%   X_manouevre = the required state that question 5 or 6 asks to keep
%   constant.
%    
% Function Variables
%   delta_T = THRUST
%   delta_e = elevator tab deflection
%   delta_a = aileron control 
%   delta_r = rudder control 
%
%   
% OUTPUTS
%   U_k is the control input required to maintain a coordinated manouevre
%
% Inga Leinasars, 02/05/2021
% Jason Iredale, 3/5/2021
% Jason Iredale, 7/05/2021 
% (updated) Inga Leinasars, Martin Shannon, Ashleigh Rattray, Jason Iredale
% Jason Iredale (updated for assignment 4) 27/5/2021 1916
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [U, U_impulse] = Controls(X_in, U_in, T, AIRCRAFT, OPERATION, ENVIRONMENT)

%% UNPACK

g       = ENVIRONMENT.gravity;

m       = AIRCRAFT.Inertia.m;
S       = AIRCRAFT.Geom.S;

operationName = OPERATION.name;



%% Initial Calculations

% U = [delta_T; delta_e; delta_a; delta_r];

[~, Q] = FlowProperties(X_in, ENVIRONMENT);
W = g*m;
CL = W/(Q*S);
VT = sqrt(X_in(1)^2 + X_in(2)^2 + X_in(3)^2);

% Use all this information to make an array of the function output U.
% This array is currently the the trim condition.
U = U_in.*ones(4,length(T));
    
% Declare the control inputs during the impulse.
U_impulse = U_in;

% Declare the delay of the impulse
T_delay = 10;

%% Manoeuvre Selection
if strcmp(operationName, 'Trim')
    % Manoeuvre 0: Trim Case

    T_impulse = 0;
    % Do nothing
    
elseif strcmp(operationName, 'Elevator Impulse')
    % Manoeuvre 1: Elevator Deflection
    
    % 5 deg elevator impulse
    U_impulse(2) = 5*(pi/180) + U_in(2);
    % Declare the time-length of the impulse
    T_impulse = 0.5;
    
elseif strcmp(operationName, 'Aileron Impulse')
    % Manoeuvre 2: Aileron Deflection
    
    % 5 deg aileron impulse
    U_impulse(3) = 5*(pi/180);

    % Declare the time-length of the impulse
    T_impulse = 0.5;
    
elseif strcmp(operationName, 'Rudder Impulse')
    % Manoeuvre 3: Rudder Deflection

    % 5 deg rudder impulse
    U_impulse(4) = 5*(pi/180);
    % Declare the time-length of the impulse
    T_impulse = 0.5;    

else
    % Invalid User Response
    error('Controls.m : invalid operation manouevre')
end


% Finds the element index of U during the impulse
impulse_index = find((T>=T_delay).*(T<=T_delay+T_impulse));

% finds the number of elements during the impulse
impulse_size = sum((T>=T_delay).*(T<=T_delay+T_impulse));

% Overwrite the impulse cotrols into the output array
U(:,impulse_index) = U_impulse.*ones(4,impulse_size);

end

