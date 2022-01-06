%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X = Integrate(X_initial, U, T, x_cg)
% The function integrates states using uses Runge-Kutta 4. It also
% normalises the quaternions.
%
% Integrates the dynamics of a 13-element initial condition (X_initial) for
% an array of inputs, 4-rows k-columns (U) and a time vector (T).
%
% X_initial can be a row or column vector. U must have inputs as rows and
% time as columns. T can be a row or coumn vector.
%
% CALLED FUNCTIONS:
% StateRates.m
%
% Jason Iredale 30/04/2021
% Jason Iredale 03/05/2021 2120
% Jason Iredale 13/05/2021 1823
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X, X_dot] = IntegrateDynamics(X_initial, U, T, FD)

% Find memory for the magnitude of the quaternions. they should always be
% equal to (or close to) 1.

X_dot = zeros(13,length(T));

X = zeros(13,length(T));

% Set the initial condition;
X(:,1) = X_initial;

% Begin the Integration Method
for k = 2:length(T)
    
    % Find the time-step
    DT = T(k)-T(k-1);

    % RK4 Terms
    
    %X_dot = zeros(13,1);
    f1 = DT*StateRates(X(:,k-1),        U(:,k-1),X_dot(:,k), FD);
    f2 = DT*StateRates(X(:,k-1)+0.5*f1,	U(:,k-1),X_dot(:,k), FD);
    f3 = DT*StateRates(X(:,k-1)+0.5*f2,	U(:,k-1),X_dot(:,k), FD);
    f4 = DT*StateRates(X(:,k-1)+f3,    	U(:,k-1),X_dot(:,k), FD);
    
    % Normalise the quaternions
    quaternion = X(7:10,k);
    X(7:10,k) = quaternion/norm(quaternion);
    
    % Calculate Next-state
    X(:,k) = X(:,k-1) + (1/6)*(f1+2*f2+2*f3+f4);
    
    X_dot(:,k) = (X(:,k)-X(:,k-1))/DT;

end

% Apppend VT, alpha and beta to the states array
[V,alpha,beta] = AeroAngles(X);
X(14,:) = V;
X(15,:) = beta;
X(16,:) = alpha;

% Append the Euler Angles
EUL = Q2E(X);
X(17,:) = EUL(1,:);
X(18,:) = EUL(2,:);
X(19,:) = EUL(3,:);

end