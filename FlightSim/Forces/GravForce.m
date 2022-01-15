%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GF = GravForce(X_k)
% Transforms the gravity vector from earth-frame to body-frame.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function GF = GravForce(X_k, ENVIRONMENT)

%% Unpack

g = ENVIRONMENT.gravity;

%   obtaining the body to earth coefficient.
C_be = DCM(X_k);

%   Multiplying the body toearth coefficient by the gravity force vector.
GF = C_be*[0;
           0;
           g];

end

