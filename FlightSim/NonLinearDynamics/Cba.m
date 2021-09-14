function C = Cba(X_k)

alpha = X_k(3);
beta = X_k(2);

C = C_y(alpha)*C_z(-beta);
end