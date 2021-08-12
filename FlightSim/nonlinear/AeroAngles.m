%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [V,alpha,beta] = AeroAngles(X_k)
% Returns the airspeed from cartesian states to magnitude and aero-angles.
% 
% CALLED FUNTIONS:
%   n/a

% Ashleigh Rattray 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [V,alpha,beta] = AeroAngles(X_k)

% Separate Vector Components 
    u = X_k(1,:); 
    v = X_k(2,:); 
    w = X_k(3,:); 
% Relative Wind/True Airspeed 
    term = u.^2 + v.^2 + w.^2; 
    V = sqrt(term); 
% AoA
    alpha = atan(w./u); 

% Beta (Angle between movement direction and air velocity direction)
% --> Sideslip angle 
    beta = atan(v./u); 
        
end 