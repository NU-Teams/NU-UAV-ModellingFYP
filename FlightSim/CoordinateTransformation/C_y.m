function CosineMatrix_y = C_y(theta)

    CosineMatrix_y =   [cos(theta)  0 -sin(theta);...
                        0           1       0;...
                        sin(theta)  0 cos(theta)];

end