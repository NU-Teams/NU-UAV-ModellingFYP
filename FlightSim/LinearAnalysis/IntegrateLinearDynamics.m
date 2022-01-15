%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [X, X_dot, A, B] = Integration(t, x0, AXIS, U, FD)
%
% This function used the euler integration method to integrate the linear
% equations of motion with respect to the axis being analysed. 

% This function calls either the longitudinal or the lateral state rates
% function to calculate the aircraft's response to a control input.
% Therefore this function assumes that the state rates functions have been
% separated regarding axis. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X_lin, X_dot_lin] = IntegrateLinearDynamics(T, x0_13, U, AIRCRAFT, ENVIRONMENT)

%% UNPACK

VW_e = ENVIRONMENT.Wind.speed*[cos(ENVIRONMENT.Wind.bearing);
                               sin(ENVIRONMENT.Wind.bearing);
                               0];


% Define first arugument of the answer as the intial conditions

CHI0                = Q2E(x0_13);
x0_12               = [x0_13(1:3);
                       x0_13(4:6);
                       CHI0;
                       x0_13(11:13)];

X_dot_lin           = zeros(12,length(T));

X_tilde             = zeros(12,1);
U_tilde             = U - U(:,1).*ones(4,length(T));
A                   = AIRCRAFT.A;
B                   = AIRCRAFT.B;

X_lin(:,1)          = X_tilde;
U                   = U_tilde;

for i = 2:length(T)
    DT = T(i) - T(i-1);

    f1 = DT*LinearDynamics(X_lin(:,i-1),         U(:,i-1), A, B); 
    f2 = DT*LinearDynamics(X_lin(:,i-1)+0.5*f1,  U(:,i-1), A, B); 
    f3 = DT*LinearDynamics(X_lin(:,i-1)+0.5*f2,  U(:,i-1), A, B); 
    f4 = DT*LinearDynamics(X_lin(:,i-1)+f3,      U(:,i-1), A, B); 
    X_lin(:,i) = X_lin(:,i-1) + (1/6)*(f1 + 2*f2 + 2*f3 + f4);

    
    % Fix the x state not integrating the offset values
    X_lin(10:11,i)	= X_lin(10:11,i) + (x0_12(1:2)-VW_e(1:2))*DT;
    
    % Log the differentials
    X_dot_lin(:,i)	= (X_lin(:,i) - X_lin(:,i-1))/DT;
end
    
% Move back to Equilibrium point
X_lin = X_lin + x0_12.*ones(length(X_lin(:,1)), length(X_lin(1,:)));

% Apppend VT, alpha and beta to the states array
[VT, beta, alpha]   = AeroAngles(X_lin);
X_lin(13,:)         = VT;
X_lin(14,:)         = beta;
X_lin(15,:)         = alpha;

end 