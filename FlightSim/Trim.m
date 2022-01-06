%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [X_output, U_output] = Trim(VT, h, x_cg)
% Finds the trim setting for level flight
%	E2Q(X_k)
%	StateRates(X_k, U_k, X_dot, x_cg)
%       - Initialisation(x_cg)
%       - DCM(X_k)
%       - AeroAngles(X_k)
%       - GravForce(X_k)
%           -- DCM(X_k)
%       - PropForces(X_k, U_k, x_cg)
%           -- Initialisation(x_cg)
%           -- FlowProperties(X_k)
%               --- AeroAngles(X_k)
%           -- AeroAngles(X_k)
%       - BodyForces(X_k, U_k, X_dot, x_cg)
%           -- Initialisation(x_cg)
%           -- DCM(X_k)
%           -- FlowProperties(X_k)
%               --- AeroAngles(X_k)
%           -- WindForces(X_k,U_k, X_dot, x_cg)
%               --- Initialisation(x_cg)
%               --- AeroAngles(X_k)
%               --- AngularRates(X_k, X_dot)
%                   ---- AeroAngles(X_k)
%           -- AngularRates(X_k, X_dot)
%           -- AeroAngles(X_k)
%
% Inga and Ashleigh 06/05/2021
% Jason             07/05/2021
% Inga, Ash, Jason  07/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_output, U_output] = Trim(FD)
%% Units
kts = 1.944;
deg = 180/pi;


%% Unpack input
VT = FD.VT;
alt = FD.alt;


%% Initialisation 
% Find the Equilibrium Point at the appropriate local 0-point

if VT >= 200/kts
    % Initial aero-angles assumption for high speed
    alpha_0 = 3/deg;
elseif VT < 200/kts
    % Initial aero-angles assumption for low speed
    alpha_0 = 9/deg;
end 

% Assume controls
delta_T_0 = 0.5;
delta_e_0 = 0;

% Max nmber of while loop iterations
max_approx_attempts = 100;

% Declare variables
EA = zeros(3,max_approx_attempts);
X = [0,0,0, 0,0,0, 0,0,0,0, 0,0,alt]'; 
U = [0 0 0 0]';
x_dot = zeros(length(X(:,1)),max_approx_attempts);


%% Lateral Work
    psi         = FD.bearing;
    beta_0      = FD.beta;
    
    W = FD.Inertia.m*FD.Inertia.g;
    [sigma, ~] = FlowProperties(X);
    rho = sigma*1.225;
    CL = W./(0.5*rho*(VT^2)*FD.Geom.S);
    
    matrix = [CL 0            FD.Aero.Cydr;
              0  FD.Aero.Clda FD.Aero.Cldr;
              0  FD.Aero.Cnda FD.Aero.Cndr];
          
    func_lateral = @(beta) (matrix^-1)*((-beta)*[FD.Aero.Cyb; FD.Aero.Clb; FD.Aero.Cnb]);
    
    %soln_0 = func_lateral(beta_0);
    %delta_a_0 = soln_0(2);
    %delta_r_0 = soln_0(3);

    
xlon(:,1) = [alpha_0; delta_T_0; delta_e_0];

%% Prep for the While loop
% Initialise the amount of perturbation of the states to get a slope
nudge_factor = 0.01;

% Index for the while loop
k = 0;

% Newton-Method Tolerance
tol = 1e-8; % suggested value 
error = 1;

while error>=tol
%% step 1: Set states & inputs we can easily solve
    
    % Set the indexing to record iterations of the while loop
    k = k+1;
    
    % Unpack
    alpha   = xlon(1,k);
    delta_T = xlon(2,k);
    delta_e = xlon(3,k);

    beta = beta_0;
    
    
    % Define the obvious parts of the state and input vector
    
    %C_ba = C_y(alpha)*C_z(-beta);      % transform airflow-frame to body-frame
    %X(1:3,k) = C_ba*[VT; 0; 0];        % body velocities
    X(1,k) = VT*cos(alpha)*cos(beta);   % u
    X(2,k) = VT*sin(beta);              % v
    X(3,k) = VT*sin(alpha)*cos(beta);   % w
    X(13,k) = alt;                      % z_e
        
%% step 2: Set Angle-states we can easily solve

    gamma   = 0;                        % aircraft is not climbing
    theta   = alpha + gamma;
    
    soln    = func_lateral(beta);
    phi     = soln(1);
    delta_a = soln(2);
    delta_r = soln(3);
    
    U(:,k) = [delta_T; delta_e; delta_a; delta_r];

    EA(:,k) = [phi; theta; psi];

    % Convert to Quaternions
    quaternions = E2Q(EA(:,k));    
    X(7:10,k) = quaternions;

    % We now have: X = [ {V}, {omega}, {quaternions}, 0, 0, 0 ]
    %              U = [ {deltas} ] 
    %
    % Now we have to verify we have the right values
%% step 3: Solve x_dot

    % Solve the Dynamic Equation
    x_dot(:,k) = StateRates(X(:,k), U(:,k), zeros(13,1), FD);
    
    xLon_dot(:,k) = (x_dot([1 3 5], k))';
    
%% step 4: Finding the Jacobian
    
    Jacob = zeros(3, 3);
    
    for state = 1:3
        
        % re-Initialise Variables
        xLon_nudge = xlon(:,k);
        X_nudge = X(:,k);
        U_nudge = U(:,k);
        
        % Perturb (ie nudge) the state
        xLon_nudge(state) = xlon(state,k) + nudge_factor;
            
        % recalculate any state dependent on alpha
        X_nudge(1) = VT*cos(xLon_nudge(1))*cos(beta);   % u
        X_nudge(2) = VT*sin(beta);                      % v
        X_nudge(3) = VT*sin(xLon_nudge(1))*cos(beta);   % w
        
        U_nudge(1) = xLon_nudge(2);          % delta_T
        U_nudge(2) = xLon_nudge(3);          % delta_e

        
        % Dynamics Equation
        x_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

        xLon_dot_nudge = x_dot_nudge([1 3 5]);
        
        % The Jacobian is the slope of the x_dot = f(x) curve. This is done
        % numerically here
        Jacob(:,state) = (xLon_dot_nudge - xLon_dot(:,k)) ./ (xLon_nudge(state) - xlon(state,k));
    end

%% step 5 The Newton Rhapson Method

    % The Newton_Rhapson Equation: x_(k+1) = x_k - f(x_k)/slope
    xlon(:,k+1) = xlon(:,k) - (Jacob^-1)*xLon_dot(:,k);
    
%% step 6 Either loop back or define outputs and escape function

    error = sum(abs(xlon(:,k+1) - xlon(:,k)));
    if error<tol
        X_output = X(:,k);
        U_output = U(:,k);
    end
    
    if k>max_approx_attempts
        disp('\nTrim.m : Too many iterations and is likely stuck in an infinate for loop\n');
        error = 0;
    end
    
    
end

% For completeness
x_dot(:,k+1) = StateRates(X(:,k), U(:,k), zeros(13,1), FD);
xLon_dot(:,k+1) = (x_dot([1 3 5], k+1))';



end

