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

% retains thrust and elevator
T_B_4to2 = [1 0;
            0 1;
            0 0;
            0 0];

B_5 = T_A_12to5*B*T_B_4to2;

B_10 = (T_A_12to5')*B_5*(T_B_4to2');
end