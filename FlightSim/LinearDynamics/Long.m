%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [JacobA, JacobB] = Long(X_bar, U_bar, x_cg)
% Finds the longitudinal Jacobian of states [u w q theta z] and returns the
% A and B Matrix for linear dynamics.
% This only works for rectilinear steady-level flight (as i used theta
% algebraically).
%
% CALLED FUNCTIONS:
% StateRates
%
% Jason Iredale 27/05/2021 1904
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [JacobA, JacobB] = Long(X_bar, U_bar, FD)
%% Initialisation 

% bulk of the longitudinal states. Theta won't be passed because it's hard
% to change it to quaternion and also it's really easy to calculate.

u_init = X_bar(1);
w_init = X_bar(3);
q_init = X_bar(5);
z_init = X_bar(13);

delta_T = U_bar(1);
delta_e = U_bar(2);

% recall: xbar = [u; w; q; theta; z_e]
xlong = [u_init; w_init; q_init; 0; z_init];
ulong(:,1) = [delta_T; delta_e];

% Initialise the amount of perturbation of the states to get a slope
nudge_factor = 0.01;


%% step 1: Solve x_dot

% Solve the Dynamic Equation
x_dot = StateRates(X_bar, U_bar, zeros(13,1), FD);

% xbar_dot = [u_dot; w_dot ; q_dot; theta_dot; z_e_dot]
xlong_dot = [x_dot(1); x_dot(3); x_dot(5); 0; x_dot(13)];
    
%% step 3: Finding the Jacobian

JacobA = zeros(5, 5);
JacobB = zeros(5, 2);

for stateA = [1 2 3 5]

    % re-Initialise Variables
    xlong_nudge = xlong;
    X_nudge = X_bar;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    xlong_nudge(stateA) = xlong(stateA) + nudge_factor;

    X_nudge(1) = xlong_nudge(1);
    X_nudge(3) = xlong_nudge(2);
    X_nudge(5) = xlong_nudge(3);
    X_nudge(13) = xlong_nudge(5);

    % Dynamics Equation
    x_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

    % xbar_dot = [u_dot; w_dot ; q_dot; theta_dot; z_e_dot]
    xlong_dot_nudge = [x_dot_nudge(1); x_dot_nudge(3); x_dot_nudge(5); 0; x_dot_nudge(13)];

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobA(:,stateA) = (xlong_dot_nudge - xlong_dot) ./ (xlong_nudge(stateA) - xlong(stateA));
end

% Variables for theta_dot and theta. StatesRates takes in quaternions so
% finding a jacobian in terms of theta is too hard. Just take the algebraic
% approach here.
JacobA(4, 3) = 1;                   % d(theta_dot)/d(theta) = q
JacobA(1, 4) = -FD.Inertia.g;       % d(u_dot)/d(theta) = -g*theta

for stateB = 1:2

    % re-Initialise Variables
    ulong_nudge = ulong;
    X_nudge = X_bar;
    U_nudge = U_bar;

    % Perturb (ie nudge) the state
    ulong_nudge(stateB) = ulong(stateB) + nudge_factor;

    U_nudge(1) = ulong_nudge(1);
    U_nudge(2) = ulong_nudge(2);

    % Dynamics Equation
    x_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

    xlong_dot_nudge = [x_dot_nudge(1); x_dot_nudge(3); x_dot_nudge(5); 0; x_dot_nudge(13)];

    % The Jacobian is the slope of the x_dot = f(x) curve. This is done
    % numerically here
    JacobB(:,stateB) = (xlong_dot_nudge - xlong_dot) ./ (ulong_nudge(stateB) - ulong(stateB));
end

end
