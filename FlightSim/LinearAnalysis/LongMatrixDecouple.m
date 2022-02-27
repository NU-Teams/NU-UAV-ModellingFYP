function [A_10, B_10, A_5, B_5] = LongMatrixDecouple(A, B)

% retains states u, w, p, theta, z
T_A_12to5 = [1 0 0, 0 0 0, 0 0 0, 0 0 0;
             0 0 1, 0 0 0, 0 0 0, 0 0 0;
             0 0 0, 0 1 0, 0 0 0, 0 0 0;
             0 0 0, 0 0 0, 0 1 0, 0 0 0;
             0 0 0, 0 0 0, 0 0 0, 0 0 1];

A_5 = T_A_12to5*A*(T_A_12to5');

% regenerates a 10x10 A matrix (x and y removed) to create a single
% decoupled A-matrix.
T_A_10to5 = [1 0 0, 0 0 0, 0 0 0, 0;
             0 0 1, 0 0 0, 0 0 0, 0;
             0 0 0, 0 1 0, 0 0 0, 0;
             0 0 0, 0 0 0, 0 1 0, 0;
             0 0 0, 0 0 0, 0 0 0, 1];

A_10 = (T_A_10to5')*A_5*T_A_10to5;

% retains thrust, elevator and flaps
T_B_5to3 = [1 0 0;
            0 1 0;
            0 0 0;
            0 0 0;
            0 0 1];

B_5 = T_A_12to5*B*T_B_5to3;

B_10 = (T_A_12to5')*B_5*(T_B_5to3');
end