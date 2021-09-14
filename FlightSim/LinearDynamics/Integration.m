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

function [X, X_dot] = Integration(T, x0, U, FD)

% Define first arugument of the answer as the intial conditions

X_tilde = zeros(13,1);
U_tilde = U - U(:,1).*ones(4,length(T));
A = FD.A;
B = FD.B;

X(:,1)	= X_tilde;
U       = U_tilde;

for i = 2:length(T)
    h = T(i) - T(i-1);

    f1 = h*LinearDynamics(X(:,i-1),         U(:,i-1), A, B); 
    f2 = h*LinearDynamics(X(:,i-1)+0.5*f1,  U(:,i-1), A, B); 
    f3 = h*LinearDynamics(X(:,i-1)+0.5*f2,  U(:,i-1), A, B); 
    f4 = h*LinearDynamics(X(:,i-1)+f3,      U(:,i-1), A, B); 
    X(:,i) = X(:,i-1) + (1/6)*(f1 + 2*f2 + 2*f3 + f4);
    
    % Normalise the quaternions
    quaternion = X(7:10,i) + x0(7:10);
    X(7:10,i) = quaternion/norm(quaternion) - x0(7:10);
    
    % Log the differentials
    X_dot(:,i) = (X(:,i) - X(:,i-1))/h;
end
    
% Move back to Equilibrium point
X = X + x0.*ones(length(X(:,1)), length(X(1,:)));

end 