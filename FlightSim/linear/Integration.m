%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [X, X_dot, A, B] = Integration(t, x0, AXIS, U, FD)
%
% This function used the euler integration method to integrate the linear
% equations of motion with respect to the axis being analysed. 

% This function calls either the longitudinal or the lateral state rates
% function to calculate the aircraft's response to a control input.
% Therefore this function assumes that the state rates functions have been
% separated regarding axis. 

% INPUTS
% x0 = the 13x1 vector generated from the trim function
% t = time vector
% U = the control inputs [delta_T ; delta_e ; delta_a ; delta_r]
% x_cg = the CoG for the scenario being assessed

% FUNCTION CALLS 
% Aero_Angles(x0) = calculates the initial alpha and beta needed for
% integration
% Q2E(x0) = calculates the initial euler angle positions needed for
% specific axis integration/state rates equations. 

% OUTPUTS


% Ashleigh Rattray 25/05/2021 2205
% Ashleigh Rattray 27/05/2021 1813
% Jason Iredale    28/05/2021 2000
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X, X_dot] = Integration(t, x0, AXIS, U, FD)


% Calculate euler angles from the trim state
    Euler_Angles = Q2E(x0); 
    phi     = Euler_Angles(1);
    theta   = Euler_Angles(2);
    psi     = Euler_Angles(3);
    
% Calculate alpha and beta
    [~, alpha, beta] = AeroAngles(x0); 

% Set control inputs to be used, and initial values to be used 
    U_long = U(1:2,:);  
    x0_long = [x0(1) ; x0(3) ; x0(5) ; theta ; x0(13)]; 
    U_lat = U(3:4,:); 
    x0_lat = [beta; x0(4) ; x0(6) ; phi ; psi]; 

% Split the 
% Define first arugument of the answer as the intial conditions

    if AXIS == "long"
        X_tilde = x0_long - x0_long;
        U_tilde = U_long - U_long(:,1).*ones(2,length(t));
        A = FD.Long.A;
        B = FD.Long.B;
    elseif AXIS == "lat"
        X_tilde = x0_lat - x0_lat;
        U_tilde = U_lat - U_lat(:,1).*ones(2,length(t));
        A = FD.Lat.A;
        B = FD.Lat.B;
    elseif AXIS == "both"
        X_tilde = zeros(13,1);
        U_tilde = U - U(:,1).*ones(4,length(t));
        A = FD.A;
        B = FD.B;
    else
        disp('Invalid axis chosen')
        return
    end
    
    X(:,1)	= X_tilde;
    U       = U_tilde;
    
    for i = 2:length(t)
        h = t(i) - t(i-1);
        
        f1 = h*LinearDynamics(X(:,i-1),         U(:,i-1), A, B); 
        f2 = h*LinearDynamics(X(:,i-1)+0.5*f1,  U(:,i-1), A, B); 
        f3 = h*LinearDynamics(X(:,i-1)+0.5*f2,  U(:,i-1), A, B); 
        f4 = h*LinearDynamics(X(:,i-1)+f3,      U(:,i-1), A, B); 
        X(:,i) = X(:,i-1) + (1/6)*(f1 + 2*f2 + 2*f3 + f4); 
        X_dot(:,i) = (X(:,i) - X(:,i-1))/h;
    end
    
    % Move back to Equilibrium point
    
    if AXIS == "long"
        X = X + x0_long.*ones(length(X(:,1)), length(X(1,:)));
    elseif AXIS == "lat"
        X = X + x0_lat.*ones(length(X(:,1)), length(X(1,:)));
    elseif AXIS == "both"
        X = X + x0.*ones(length(X(:,1)), length(X(1,:)));
    else
        disp('Invalid axis chosen')
    end
end 