function CosineMatrix_x = C_x(phi)
% Jason Iredale, 30/04/2021

%% FUNCTION SUMMARY

% INPUTS:
    % phi
    
% FUNCTION VARIABLES
       
% OUTPUTS:
    % CosineMatrix_x 

%% CODE
CosineMatrix_x =   [1   0       0;...
                    0 cos(phi)  sin(phi);...
                    0 -sin(phi) cos(phi)];

end