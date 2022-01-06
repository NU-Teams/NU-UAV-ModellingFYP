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

function [X, X_dot] = IntegrateLinearDynamics(T, x0, U, FD)

% Define first arugument of the answer as the intial conditions

X_dot = zeros(13,length(T));

X_tilde = zeros(13,1);
U_tilde = U - U(:,1).*ones(4,length(T));
A = FD.A;
B = FD.B;

X(:,1)	= X_tilde;
U       = U_tilde;

for i = 2:length(T)
    DT = T(i) - T(i-1);

    f1 = DT*LinearDynamics(X(:,i-1),         U(:,i-1), A, B); 
    f2 = DT*LinearDynamics(X(:,i-1)+0.5*f1,  U(:,i-1), A, B); 
    f3 = DT*LinearDynamics(X(:,i-1)+0.5*f2,  U(:,i-1), A, B); 
    f4 = DT*LinearDynamics(X(:,i-1)+f3,      U(:,i-1), A, B); 
    X(:,i) = X(:,i-1) + (1/6)*(f1 + 2*f2 + 2*f3 + f4);
    
    % Normalise the quaternions
    quaternion = X(7:10,i) + x0(7:10);
    X(7:10,i) = quaternion/norm(quaternion) - x0(7:10);
    
    % Fix the x state not integrating the offset values
    X(11:12,i) = X(11:12,i) + (x0(1:2)-FD.VW_e(1:2))*DT;
    
    % Log the differentials
    X_dot(:,i) = (X(:,i) - X(:,i-1))/DT;
end
    
% Move back to Equilibrium point
X = X + x0.*ones(length(X(:,1)), length(X(1,:)));

% Apppend VT, alpha and beta to the sttes array
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