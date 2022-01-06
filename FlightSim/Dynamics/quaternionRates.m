function [quat_dot] = quaternionRates(X_k)

%% initialisation of states & variables from input vectors

    p = X_k(4);
    q = X_k(5);
    r = X_k(6);
    q0 = X_k(7);
    q1 = X_k(8);
    q2 = X_k(9); 
    q3 = X_k(10);

    quatVector = [q0; q1; q2; q3];

%% 4 attitude rates (using quarternions)
    
    %{
    q0_dot = -(1/2)*(q1*p + q2*q + q3*r);
    q1_dot =  (1/2)*(q0*p - q3*q + q2*r);
    q2_dot =  (1/2)*(q3*p + q0*q - q1*r);
    q3_dot = -(1/2)*(q2*p - q1*q - q0*r);
    %}
    
    quat_pqrMatrix = (1/2)*[0 -p -q -r;
                            p  0  r -q;
                            q -r  0  p;
                            r  q -p  0];
    
    quat_dot = quat_pqrMatrix*quatVector;

end