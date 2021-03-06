%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CosineMatrix = DCM(X_k)
% Finds the body to earth and vice-verse transformation matrix for a state.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function C_be = DCM(X_k)
%% CODE

% Declaring the quaternions from the state variable
q0 = X_k(7,:);
q1 = X_k(8,:);
q2 = X_k(9,:);
q3 = X_k(10,:);

% Finding the elements of the DCM Matrix
% row 1
l1 = q0.^2 + q1.^2 - q2.^2 - q3.^2;
l2 = 2*(q1.*q2 + q0.*q3);
l3 = 2*(q1.*q3 - q0.*q2);
% row 2
m1 = 2*(q1.*q2 - q0.*q3);
m2 = q0.^2 - q1.^2 + q2.^2 - q3.^2;
m3 = 2*(q2.*q3 + q0.*q1);
% row 3
n1 = 2*(q0.*q2 + q1.*q3);
n2 = 2*(q2.*q3 - q0.*q1);
n3 = q0.^2 - q1.^2 - q2.^2 + q3.^2;

% The DCM that transforms from LVLH to BODY
C_be = [l1 l2 l3;
        m1 m2 m3;
        n1 n2 n3];
       

end