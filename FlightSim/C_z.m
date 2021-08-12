function CosineMatrix_z = C_z(psi)
% Jason Iredale, 30/04/2021
% Note: Jason provided his assignment 1 code after comparison with Inga,
% Marty and Ash where all 4 individual codes were the same.
%% FUNCTION SUMMARY

% INPUTS:
        % psi
    
% FUNCTION VARIABLES
       
% OUTPUTS:
    % CosineMatrix_z

%% CODE

CosineMatrix_z =   [cos(psi)    sin(psi)    0;...
                    -sin(psi)   cos(psi)    0;...
                        0           0       1];

end