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
function [X_output, X_dot] = IntegrateDynamics(X_initial, U, T, AIRCRAFT, ENVIRONMENT)

% Find memory for the magnitude of the quaternions. they should always be
% equal to (or close to) 1.

X_dot               = zeros(13,length(T));

X                   = zeros(13,length(T));

% geo-coordinates data (todo: put the ic is the app.)
geo(:, 1)           = [-32.890219*(pi/180); 151.702862*(pi/180); -X_initial(end)];
R                   = 6371e+3;

% Set the initial condition;
X(:,1)              = X_initial;

% Begin the Integration Method
for k = 2:length(T)
    
    % Find the time-step
    DT              = T(k)-T(k-1);

    % RK4 Terms
    
    %X_dot = zeros(13,1);
    f1 = DT*StateRates(X(:,k-1),        U(:,k-1),X_dot(:,k), AIRCRAFT, ENVIRONMENT);
    f2 = DT*StateRates(X(:,k-1)+0.5*f1,	U(:,k-1),X_dot(:,k), AIRCRAFT, ENVIRONMENT);
    f3 = DT*StateRates(X(:,k-1)+0.5*f2,	U(:,k-1),X_dot(:,k), AIRCRAFT, ENVIRONMENT);
    f4 = DT*StateRates(X(:,k-1)+f3,    	U(:,k-1),X_dot(:,k), AIRCRAFT, ENVIRONMENT);
    
    % Normalise the quaternions
    quaternion      = X(7:10,k);
    X(7:10,k)       = quaternion/norm(quaternion);
    
    % Calculate Next-state
    X(:,k)          = X(:,k-1) + (1/6)*(f1+2*f2+2*f3+f4);
    
    X_dot(:,k)      = (X(:,k)-X(:,k-1))/DT;
    
    % EARTH coordinates - used for mapping (todo: make this a dynamic eqn?)
    geo(3, k)       = -(X(end,   k) - X(end  , k-1))                                  + geo(3, k-1);
    geo(1, k)       =  (X(end-2, k) - X(end-2, k-1))/( R + geo(3, k)                ) + geo(1, k-1);
    geo(2, k)       =  (X(end-1, k) - X(end-1, k-1))/((R + geo(3, k))*cos(geo(1, k))) + geo(2, k-1);

end

% Apppend VT, alpha and beta to the states array
[V, beta, alpha]	= AeroAngles(X);
aerodynamicAngles   = [V; beta; alpha];

% swap quaternions for euler angles
EUL                 = Q2E(X);

% latitude longitude coordinates
geo(1:2, :)         = geo(1:2, :)*180/pi

X_output = [X(1:3,:);           % body velocities
            X(4:6,:);           % rotation rates
            EUL;                % attitudes
            X(11:13,:);         % position (LVLH)
            aerodynamicAngles;  % different kind of velocities
            geo];               % geo coordinates
end