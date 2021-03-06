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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_output, X_dot] = IntegrateDynamics(X_initial, U, T, AIRCRAFT, OPERATION, ENVIRONMENT)

% Find memory for the magnitude of the quaternions. they should always be
% equal to (or close to) 1.

X_dot               = zeros(13,length(T));
X_dot(:, 2:end) = NaN;

X                   = zeros(13,length(T));
X(:, 2:end) = NaN;

% Set the initial condition;
X(:,1)              = X_initial;

% geo-coordinates data
geo                 = zeros(3, length(T));
geo(:, 2:end) = NaN;
azimuth0            = OPERATION.Trim.bearing;
geo(:, 1)           = [OPERATION.latitude0;
                       OPERATION.longitude0;
                       azimuth0];


% Begin the Integration Method
for k = 2:length(T)
    
    % Find the time-step
    DT              = T(k)-T(k-1);

    % RK4 Terms
    f1 = DT*StateRates(X(:,k-1),        U(:,k-1), X_dot(:,k-1), AIRCRAFT, ENVIRONMENT);
    f2 = DT*StateRates(X(:,k-1)+0.5*f1,	U(:,k-1), X_dot(:,k-1), AIRCRAFT, ENVIRONMENT);
    f3 = DT*StateRates(X(:,k-1)+0.5*f2,	U(:,k-1), X_dot(:,k-1), AIRCRAFT, ENVIRONMENT);
    f4 = DT*StateRates(X(:,k-1)+f3,    	U(:,k-1), X_dot(:,k-1), AIRCRAFT, ENVIRONMENT);
    
    % Calculate Next-state
    X(:,k)          = X(:,k-1) + (1/6)*(f1+2*f2+2*f3+f4);
    
    X_dot(:,k)      = (X(:,k)-X(:,k-1))/DT;
        
    % Normalise the quaternions
    quaternion      = X(7:10,k);
    X(7:10,k)       = quaternion/norm(quaternion);
    
    % EARTH coordinates - used for mapping
    % distance
    x               = X(11, k)-X(11, k-1);
    y               = X(12, k)-X(12, k-1);
    distance        = sqrt(x^2 + y^2);
    
    % change in heading, delta = psi - (-beta)
    EUL_k           = Q2E(X(:,k));
    EUL_k_1         = Q2E(X(:,k-1));
    [~, beta_k, ~]	= AeroAngles(X(:,k)); 
    [~, beta_k_1, ~]= AeroAngles(X(:,k-1));
    d_delta         = (EUL_k(3)+beta_k) -  (EUL_k_1(3)+beta_k_1);
    geo(3, k-1)     = geo(3, k-1) + d_delta;
    
    % using Vincenty's formula to turn path along a great circle into
    % latitude longitue coodinates. May be overpowered but its the only
    % thing i found online - jason.
    geo(:,k)        = VincentyFormula(geo(:, k-1), distance);

    % Crash condiion: if the altitude is below sea level - end simulation
    if (-X(13,k)) < 0
        break        
    end
    
end

% Apppend VT, alpha and beta to the states array
[V, beta, alpha]	= AeroAngles(X);
aerodynamicAngles   = [V; beta; alpha];

% swap quaternions for euler angles
EUL                 = Q2E(X);

% latitude longitude coordinates
geo(1:2, :)         = geo(1:2, :)*180/pi;

X_output = [X(1:3,:);           % body velocities
            X(4:6,:);           % rotation rates
            EUL;                % attitudes
            X(11:13,:);         % position (LVLH)
            aerodynamicAngles;  % different kind of velocities
            geo];               % geo coordinates

end