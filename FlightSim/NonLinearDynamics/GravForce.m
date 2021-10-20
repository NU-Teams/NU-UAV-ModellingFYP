%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GF = GravForce(X_k)
% Transforms the gravity vector from earth-frame to body-frame.
%
% Marty Shannon, 7/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [GF] = GravForce(X_k, FD)

%   obtaining the body to earth coefficient.
CosineMatrix = DCM(X_k);
Cbe = CosineMatrix.Cbe;

%   Multiplying the body toearth coefficient by the gravity force vector.
GF = Cbe*[0; 0; FD.Inertia.g];

end

