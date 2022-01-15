%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [V,alpha,beta] = AeroAngles(X_k)
% Returns the airspeed from cartesian states to magnitude and aero-angles.
% 
% CALLED FUNTIONS:
%   n/a

% Ashleigh Rattray 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [V, beta, alpha] = AeroAngles(X_k)

% Separate Vector Components 
    u = X_k(1,:); 
    v = X_k(2,:); 
    w = X_k(3,:); 
    
% Relative Wind/True Airspeed 
    V = sqrt(u.^2 + v.^2 + w.^2); 
    
% AoA
    alpha = atan(w./u); 

% Beta (Angle between movement direction and air velocity direction)
% --> Sideslip angle 
    beta = asin(v./V); 
        
end 