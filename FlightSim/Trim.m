%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [X_output, U_output] = Trim(VT, h, x_cg)
% Finds the trim setting for level flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X_output, U_output] = Trim(AIRCRAFT, OPERATION, ENVIRONMENT)

%% Units
kts = 1.944;
deg = 180/pi;


%% Unpack input
VT      = OPERATION.Trim.airspeed/kts;
alt     = OPERATION.Trim.altitude;
g       = ENVIRONMENT.gravity;
rho_sl  = ENVIRONMENT.Sealevel.density;
m       = AIRCRAFT.Inertia.m;
S       = AIRCRAFT.Geom.S;
delta_0 = OPERATION.Trim.bearing/deg;
beta_0  = OPERATION.Trim.beta/deg;


% todo: add climbing in the future:
gamma   = 0;                        % aircraft is not climbing


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

X = [0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     alt];
 
U = [0;
     0;
     0;
     0];
 
x_dot = zeros(length(X(:,1)),max_approx_attempts);


%% Lateral Work
    
    W = m*g;
    [sigma, ~] = FlowProperties(X, ENVIRONMENT);
    Q = (1/2)*sigma*rho_sl*VT^2;
    CL = W./(Q*S);
    
    matrix = [CL 0                  AIRCRAFT.Aero.CYdr;
              0  AIRCRAFT.Aero.Clda AIRCRAFT.Aero.Cldr;
              0  AIRCRAFT.Aero.Cnda AIRCRAFT.Aero.Cndr];
          
    func_lateral = @(beta) (matrix^-1)*((-beta)*[AIRCRAFT.Aero.CYb;
                                                 AIRCRAFT.Aero.Clb;
                                                 AIRCRAFT.Aero.Cnb]);
    
    %soln_0 = func_lateral(beta_0);
    %delta_a_0 = soln_0(2);
    %delta_r_0 = soln_0(3);

    
longParams(:,1) = [alpha_0; delta_T_0; delta_e_0];

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
    alpha   = longParams(1,k);
    delta_T = longParams(2,k);
    delta_e = longParams(3,k);

    beta = beta_0;
    
    
    % Define the obvious parts of the state and input vector
    % velocities
    C_ba = air2body(beta, alpha);
    X(1:3, k) = C_ba*[VT;
                       0;
                       0];
    % z_e
    X(13,k) = alt;
        
%% step 2: Set Angle-states we can easily solve

    theta   = alpha + gamma;
    psi     = delta_0-beta;
    
    soln    = func_lateral(beta);
    phi     = soln(1);
    delta_a = soln(2);
    delta_r = soln(3);
    
    U(:,k) = [delta_T;
              delta_e;
              delta_a;
              delta_r];

    EA(:,k) = [phi;
               theta;
               psi];

    % Convert to Quaternions
    quaternions = E2Q(EA(:,k));    
    X(7:10,k) = quaternions;

    % We now have: X = [ {V}, {omega}, {quaternions}, 0, 0, 0 ]
    %              U = [ {deltas} ] 
    %
    % Now we have to verify we have the right values
    
    
%% step 3: Solve x_dot

    % Solve the Dynamic Equation
    x_dot(:,k) = StateRates(X(:,k), U(:,k), zeros(13,1), AIRCRAFT, ENVIRONMENT);
    
    xLon_dot(:,k) = (x_dot([1 3 5], k))';
    
    
%% step 4: Finding the Jacobian
    
    Jacob = zeros(3, 3);
    
    for state = 1:3
        
        % re-Initialise Variables
        longParams_nudge = longParams(:,k);
        X_nudge = X(:,k);
        U_nudge = U(:,k);
        
        % Perturb (ie nudge) the state
        longParams_nudge(state) = longParams(state,k) + nudge_factor;
            
        % recalculate any state dependent on alpha
        C_ba = air2body(beta, longParams_nudge(1));
        X_nudge(1:3) = C_ba*[VT; 0; 0];         % u
        
        U_nudge(1) = longParams_nudge(2);      	% delta_T
        U_nudge(2) = longParams_nudge(3);       % delta_e

        
        % Dynamics Equation
        x_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), AIRCRAFT, ENVIRONMENT);

        xLon_dot_nudge = x_dot_nudge([1 3 5]);
        
        % The Jacobian is the slope of the x_dot = f(x) curve. This is done
        % numerically here
        Jacob(:,state) = (xLon_dot_nudge - xLon_dot(:,k)) ./ (longParams_nudge(state) - longParams(state,k));
    end

    
%% step 5 The Newton Rhapson Method

    % The Newton_Rhapson Equation: x_(k+1) = x_k - f(x_k)/slope
    longParams(:,k+1) = longParams(:,k) - (Jacob^-1)*xLon_dot(:,k);
    
    
%% step 6 Either loop back or define outputs and escape function

    error = sum(abs(longParams(:,k+1) - longParams(:,k)));
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
x_dot(:,k+1) = StateRates(X(:,k), U(:,k), zeros(13,1), AIRCRAFT, ENVIRONMENT);
xLon_dot(:,k+1) = (x_dot([1 3 5], k+1))';



end


