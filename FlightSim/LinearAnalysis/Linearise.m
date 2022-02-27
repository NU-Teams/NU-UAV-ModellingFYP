%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [JacobA, JacobB] = Linearise(X_bar, U_bar, FD)
% Numerically Linearises the dynamics.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [JacobA, JacobB] = Linearise(X_bar, U_bar, AIRCRAFT, ENVIRONMENT)


CHI = Q2E(X_bar);

X_bar_lin = [X_bar(1:3);
             X_bar(4:6);
             CHI;
             X_bar(11:13)];
         
%% Initialisation 

% Initialise the amount of perturbation of the states to get a slope
nudge_factor = 0.01;

%% step 1: Solve x_dot

% Solve the Dynamic Equation
X_dot = StateRates_linear(X_bar_lin, U_bar, zeros(12,1), AIRCRAFT, ENVIRONMENT);

%% step 3: Finding the Jacobian

JacobA = zeros(12, 12);
JacobB = zeros(12, 5);

for stateA = 1:12

    % re-Initialise Variables
    X_nudge = X_bar_lin;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    X_nudge(stateA) = X_bar_lin(stateA) + nudge_factor;

    % Dynamics Equation
    X_dot_nudge = StateRates_linear(X_nudge, U_nudge, zeros(13, 1), AIRCRAFT, ENVIRONMENT);

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobA(:,stateA) = (X_dot_nudge - X_dot) ./ (X_nudge(stateA) - X_bar_lin(stateA));
end

for stateB = 1:5

    % re-Initialise Variables
    X_nudge = X_bar_lin;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    U_nudge(stateB) = U_bar(stateB) + nudge_factor;

    % Dynamics Equation
    X_dot_nudge = StateRates_linear(X_nudge, U_nudge, zeros(13, 1), AIRCRAFT, ENVIRONMENT);

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobB(:,stateB) = (X_dot_nudge - X_dot) ./ (U_nudge(stateB) - U_bar(stateB));
end

end
