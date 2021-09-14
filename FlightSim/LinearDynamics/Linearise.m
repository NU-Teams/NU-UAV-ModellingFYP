%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [JacobA, JacobB] = Linearise(X_bar, U_bar, FD)
% Numerically Linearises the dynamics.
% CALLED FUNCTIONS:
% StateRates
%
% Jason Iredale 27/05/2021 1904
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [JacobA, JacobB] = Linearise(X_bar, U_bar, FD)
%% Initialisation 

% Initialise the amount of perturbation of the states to get a slope
nudge_factor = 0.01;

%% step 1: Solve x_dot

% Solve the Dynamic Equation
X_dot = StateRates(X_bar, U_bar, zeros(13,1), FD);

%% step 3: Finding the Jacobian

JacobA = zeros(13, 13);
JacobB = zeros(13, 4);

for stateA = 1:13

    % re-Initialise Variables
    X_nudge = X_bar;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    X_nudge(stateA) = X_bar(stateA) + nudge_factor;

    % Dynamics Equation
    X_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobA(:,stateA) = (X_dot_nudge - X_dot) ./ (X_nudge(stateA) - X_bar(stateA));
end

for stateB = 1:4

    % re-Initialise Variables
    X_nudge = X_bar;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    U_nudge(stateB) = U_bar(stateB) + nudge_factor;

    % Dynamics Equation
    X_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobB(:,stateB) = (X_dot_nudge - X_dot) ./ (U_nudge(stateB) - U_bar(stateB));
end

% T_A_13to5 = [1 0 0 0 0 0 0 0 0 0 0 0 0;
%              0 0 1 0 0 0 0 0 0 0 0 0 0;
%              0 0 0 0 1 0 0 0 0 0 0 0 0;
%              0 0 0 0 0 0 0 0 0 0 0 0 0;
%              0 0 0 0 0 0 0 0 0 0 0 0 1];
% 
% A_long = T_A_13to5*JacobA*(T_A_13to5');
% A_long(4,3) = 1;
% A_long(1,4) = -FD.Inertia.g;
% 
% T_A_10to5 = [1 0 0 0 0 0 0 0 0 0;
%              0 0 1 0 0 0 0 0 0 0;
%              0 0 0 0 1 0 0 0 0 0;
%              0 0 0 0 0 0 0 1 0 0;
%              0 0 0 0 0 0 0 0 0 1];
% 
% A_10 = (T_A_10to5')*A_long*T_A_10to5;
% 
% T_B_4to2 = [1 0;
%             0 1;
%             0 0;
%             0 0];
% 
% B_long = T_A_13to5*JacobB*T_B_4to2;
% 
% B_10 = (T_A_13to5')*B_long*(T_B_4to2');

end
