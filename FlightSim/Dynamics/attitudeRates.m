function [EUL_dot] = attitudeRates(X_k_lin)

%% initialisation of states & variables from input vectors

p = X_k_lin(4);
q = X_k_lin(5);
r = X_k_lin(6);
phi = X_k_lin(7);
theta = X_k_lin(8);

omega = [p;
         q;
         r];

%% 3 attitude rates
    
    
    eulerMatrix = [1         0         -sin(theta);
                   0  cos(phi) sin(phi)*cos(theta);
                   0 -sin(phi) cos(phi)*cos(theta)];
               
    eulerMatrixInv = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                      0            cos(phi)           -sin(phi);
                      0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
    
    EUL_dot = (eulerMatrixInv)*omega;

end