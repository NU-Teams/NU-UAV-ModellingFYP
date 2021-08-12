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
function [X_output, U_output] = Trim(VT, h, FD)
%% Initialisation 

% Find the Equilibrium Point at the appropriate local 0-point
if VT >= 200/1.944
    alpha_0 = (pi/180)*3;   % Initial alpha assumption for high speed
elseif VT < 200/1.944
    alpha_0 = (pi/180)*9;   % Initial alpha assumption so low speed
end 

% Assume a elevator and thrust delta
delta_T_0 = 0.5;
delta_e_0 = 0;

% recall: xbar = [alpha; delta_T; delta_e]
xbar(:,1) = [alpha_0, delta_T_0, delta_e_0]';

% Max nmber of while loop iterations
max_approx_attempts = 1000;

theta = zeros(1,max_approx_attempts);
EA = zeros(3,max_approx_attempts);
X = [0,0,0, 0,0,0, 0,0,0,0, 0,0,h]'; 
U = [0 0 0 0]';
x_dot = zeros(length(X(:,1)),max_approx_attempts);

% Initialise the amount of perturbation of the states to get a slope
nudge_factor = 0.01;

% Index for the while loop
k = 0;

% Newton-Method Tolerance
tol = 10e-9; % suggested value 
error = 1;

while error>=tol
%% step 1: Set states & inputs we can easily solve
    
    % Set the indexing to record iterations of the while loop
    k = k+1;
    
    alpha = xbar(1,k);
    delta_T = xbar(2,k);
    delta_e = xbar(3,k);
    
    X(1,k) = VT*cos(alpha);     % u 
    X(3,k) = VT*sin(alpha);     % w
    X(13,k) = h;                % z_e
    U(1,k) = delta_T;
    U(2,k) = delta_e;

    
%% step 2: Set Angle-states we can easily solve

    % We are assuming no wind in trimmed level flight so there is
    % no bank angle and no yaw angle
    gamma = 0; % aircraft is not climbing
    
    theta(k) = alpha + gamma;
    EA(:,k) = [0; theta(k); 0];

    % Convert to Quaternions
    quaternions = E2Q(EA(:,k));
    
    % Normlise quaternions (just in case)
    quaternions = quaternions/norm(quaternions);
    
    X(7:10,k) = quaternions;

    % We now have: X = [ {V}, {omega}, {quaternions}, 0, 0, 0 ]
    %              U = [ {deltas} ] 

%% step 3: Solve x_dot

    % Solve the Dynamic Equation
    x_dot(:,k) = StateRates(X(:,k), U(:,k), zeros(13,1), FD);

    % We only care about u_dot, w_dot & q_dot because they describe the 2D
    % case of flying (again, there is no bank angle or yaw).
    
    % xbar_dot = [u_dot; w_dot ; q_dot]
    xbar_dot(:,k) = [x_dot(1,k) x_dot(3,k) x_dot(5,k)]';
    
%% step 4: Finding the Jacobian
    
    Jacob = zeros(3,3);
    
    for state = 1:3
        
        % re-Initialise Variables
        % recall: xbar = [alpha; delta_T; delta_e]
        xbar_nudge = xbar(:,k);
        X_nudge = X(:,k);
        U_nudge = U(:,k);
        
        % Perturb (ie nudge) the state
        xbar_nudge(state) = xbar(state,k) + nudge_factor;
            
        % recalculate any state dependent on alpha
        % recall: xbar = [alpha; delta_T; delta_e]
        X_nudge(1) = VT*cos(xbar_nudge(1));  % u
        X_nudge(3) = VT*sin(xbar_nudge(1));  % w
        U_nudge(1) = xbar_nudge(2);          % detla_T
        U_nudge(2) = xbar_nudge(3);          % delta_e
        
        % Dynamics Equation
        x_dot_nudge = StateRates(X_nudge, U_nudge, zeros(13, 1), FD);

        xbar_dot_nudge = [x_dot_nudge(1) x_dot_nudge(3) x_dot_nudge(5)]';
        
        % The Jacobian is the slope of the x_dot = f(x) curve. This is done
        % numerically here
        Jacob(:,state) = (xbar_dot_nudge - xbar_dot(:,k)) ./ (xbar_nudge(state) - xbar(state,k));
    end

%% step 5 The Newton Rhapson Method

    % The Newton_Rhapson Equation: x_(k+1) = x_k - f(x_k)/slope
    xbar(:,k+1) = xbar(:,k) - (Jacob^-1)*xbar_dot(:,k);
    
%% step 6 Either loop back or define outputs and escape function

    error = sum(abs(xbar(:,k+1) - xbar(:,k)));
    if error<tol
        X_output = X(:,k);
        U_output = U(:,k);
    end
    
    if k>max_approx_attempts
        disp('\nTrim.m : Too many iterations and is likely stuck in an infinate for loop\n');
        error = 0;
    end
    
    
end

x_dot(:,k+1) = StateRates(X(:,k), U(:,k), zeros(13,1), FD);
xbar_dot(:,k+1) = [x_dot(1,k+1) x_dot(3,k+1) x_dot(5,k+1)]';

%% DEBUG: plot x_dot = f(x)
% is the slope is positive it is unstable and we likely have done something
% wrong

%{
c = 1:k+1;

figure('Name','Trim.m DEBUG: x_dot = f(x)')
subplot(3,3,1)
    hold on
    grid on
    plot(xbar(1,:)*(180/pi),xbar_dot(1,:),'k','LineWidth',0.5)
    scatter(xbar(1,:)*(180/pi),xbar_dot(1,:),10,c,'LineWidth',1)
    plot(mod(xbar(1,end),2*pi)*(180/pi),xbar_dot(1,end),'xr','LineWidth',2)
    xlabel('\alpha [deg]')
    ylabel('u dot')
    colorbar
hold off

subplot(3,3,4)
    hold on
    grid on
    plot(xbar(1,:)*(180/pi),xbar_dot(2,:),'k','LineWidth',0.5)
    scatter(xbar(1,:)*(180/pi),xbar_dot(2,:),10,c,'LineWidth',1)
    plot(mod(xbar(1,end),2*pi)*(180/pi),xbar_dot(2,end),'xr','LineWidth',2)
    xlabel('\alpha [deg]')
    ylabel('w dot')
    colorbar
hold off

subplot(3,3,7)
    hold on
    grid on
    plot(xbar(1,:)*(180/pi),xbar_dot(3,:),'k','LineWidth',0.5)
    scatter(xbar(1,:)*(180/pi),xbar_dot(3,:),10,c,'LineWidth',1)
    plot(mod(xbar(1,end),2*pi)*(180/pi),xbar_dot(3,end),'xr','LineWidth',2)
    xlabel('\alpha [deg]')
    ylabel('q dot')
    colorbar
hold off
    
subplot(3,3,2)
    hold on
    grid on
    plot(xbar(2,:),xbar_dot(1,:),'k','LineWidth',0.5)
    scatter(xbar(2,:),xbar_dot(1,:),10,c,'LineWidth',1)
    plot(xbar(2,end),xbar_dot(1,end),'xr','LineWidth',2)
    xlabel('\delta T')
    ylabel('u dot')
    colorbar
hold off
    
subplot(3,3,5)
    hold on
    grid on
    plot(xbar(2,:),xbar_dot(2,:),'k','LineWidth',0.5)
    scatter(xbar(2,:),xbar_dot(2,:),10,c,'LineWidth',1)
    plot(xbar(2,end),xbar_dot(2,end),'xr','LineWidth',2)
    xlabel('\delta T')
    ylabel('w dot')
    colorbar
hold off

subplot(3,3,8)
    hold on
    grid on
    plot(xbar(2,:),xbar_dot(3,:),'k','LineWidth',0.5)
    scatter(xbar(2,:),xbar_dot(3,:),10,c,'LineWidth',1)
    plot(xbar(2,end),xbar_dot(3,end),'xr','LineWidth',2)
    xlabel('\delta T')
    ylabel('q dot')
    colorbar
hold off

subplot(3,3,3)
    hold on
    grid on
    plot(xbar(3,:)*(180/pi),xbar_dot(1,:),'k','LineWidth',0.5)
    scatter(xbar(3,:)*(180/pi),xbar_dot(1,:),10,c,'LineWidth',1)
    plot(xbar(3,end)*(180/pi),xbar_dot(1,end),'xr','LineWidth',2)
    xlabel('\delta e [deg]')
    ylabel('u dot')
    colorbar
hold off

subplot(3,3,6)
    hold on
    grid on
    plot(xbar(3,:)*(180/pi),xbar_dot(2,:),'k','LineWidth',0.5)
    scatter(xbar(3,:)*(180/pi),xbar_dot(2,:),10,c,'LineWidth',1)
    plot(xbar(3,end)*(180/pi),xbar_dot(2,end),'xr','LineWidth',2)
    xlabel('\delta e [deg]')
    ylabel('w dot')
    colorbar
hold off

subplot(3,3,9)
    hold on
    grid on
    plot(xbar(3,:)*(180/pi),xbar_dot(3,:),'k','LineWidth',0.5)
    scatter(xbar(3,:)*(180/pi),xbar_dot(3,:),10,c,'LineWidth',1)
    plot(xbar(3,end)*(180/pi),xbar_dot(3,end),'xr','LineWidth',2)
    xlabel('\delta e [deg]')
    ylabel('q dot')
    colorbar
hold off
%}

end


