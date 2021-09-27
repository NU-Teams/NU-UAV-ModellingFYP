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
function [U, U_impulse, question_str] = Controls(X_in, U_in, T, FD)
%% Prompt for Manoeuvre

% Print manouevres to command window
manouevre_str0 = '\n0: Trim';
manouevre_str1 = '\n1: elevator Impulse \n2: aileron Impulse \n3: rudder Impulse';
% manouevre_str2 = '\n4: 3g loop \n5: Steady-Heading Side-Slip \n6: 4-pt. hesitation roll';
fprintf([manouevre_str0 manouevre_str1])

% Prompt the User to pick a manouevre
prompt = '\nWhat manoeuvre (0 - 3) do you want to simulate? \n';

% Prompt the User to pick a manouevre
question = input(prompt);


%% Initial Calculations

% U = [delta_T; delta_e; delta_a; delta_r];

[density_ratio, Q] = FlowProperties(X_in);
rho = density_ratio*1.225;
W = FD.Inertia.g*FD.Inertia.m;
CL = W/(Q*FD.Geom.S);
VT = sqrt(X_in(1)^2 + X_in(2)^2 + X_in(3)^2);

% Use all this information to make an array of the function output U.
% This array is currently the the trim condition.
U = U_in.*ones(4,length(T));
    
% Declare the control inputs during the impulse.
U_impulse = U_in;

% Declare the delay of the impulse
T_delay = 10;

%% Manoeuvre Selection
if question == 0
    % Manoeuvre 0: Trim Case

    T_impulse = 0;
    % Do nothing
    question_str = 'Trimmed';
    
elseif question == 1
    % Manoeuvre 1: Elevator Deflection
    
    % 5 deg elevator impulse
    U_impulse(2) = 5*(pi/180) + U_in(2);
    % Declare the time-length of the impulse
    T_impulse = 0.5;
    question_str = 'Elevator Impulse';
    
elseif question == 2
    % Manoeuvre 2: Aileron Deflection
    
    % 5 deg aileron impulse
    U_impulse(3) = 5*(pi/180);

    % Declare the time-length of the impulse
    T_impulse = 0.5;
    question_str = 'Aileron Impulse';
    
elseif question == 3
    % Manoeuvre 3: Rudder Deflection

    % 5 deg rudder impulse
    U_impulse(4) = 5*(pi/180);
    % Declare the time-length of the impulse
    T_impulse = 0.5;
    question_str = 'Rudder Impulse';
    

else
    % Invalid User Response
    disp('Controls.m : invalid question number')
end

% Finds the element index of U during the impulse
impulse_index = find((T>=T_delay).*(T<=T_delay+T_impulse));
% finds the number of elements during the impulse
impulse_size = sum((T>=T_delay).*(T<=T_delay+T_impulse));
% Overwrite the impulse cotrols into the output array
U(:,impulse_index) = U_impulse.*ones(4,impulse_size);

end

