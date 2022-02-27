function CosineMatrix_z = C_z(psi)

    CosineMatrix_z =   [cos(psi)    sin(psi)    0;...
                        -sin(psi)   cos(psi)    0;...
                            0           0       1];

end