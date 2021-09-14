function Xdot = LinearDynamics(X_k, U_k, A, B)

Xdot = A*X_k + B*U_k;

end