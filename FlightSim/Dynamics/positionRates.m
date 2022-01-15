function position_e_dot = positionRates(X_k, ENVIRONMENT)

% Ceb is the body --> Earth Transform Matrix
C_eb = (DCM(X_k))^-1;

u = X_k(1);
v = X_k(2);
w = X_k(3);

velVector = [u;
             v;
             w];

VW_e = ENVIRONMENT.Wind.speed*[cos(ENVIRONMENT.Wind.bearing);
                               sin(ENVIRONMENT.Wind.bearing);
                               0];
    
position_e_dot = C_eb*velVector - VW_e;

end