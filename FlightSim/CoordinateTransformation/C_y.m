function CosineMatrix_y = C_y(theta)
% Jason Iredale, 30/04/2021

%% FUNCTION SUMMARY

% INPUTS:
    % theta
    
% FUNCTION VARIABLES
       
% OUTPUTS:
    % CosineMatrix_y 

%% CODE
CosineMatrix_y =   [cos(theta)  0 -sin(theta);...
                    0           1       0;...
                    sin(theta)  0 cos(theta)];

end