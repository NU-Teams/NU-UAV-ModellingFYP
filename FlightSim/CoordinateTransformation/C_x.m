function CosineMatrix_x = C_x(phi)
    
    CosineMatrix_x =   [1   0       0;...
                        0 cos(phi)  sin(phi);...
                        0 -sin(phi) cos(phi)];

end