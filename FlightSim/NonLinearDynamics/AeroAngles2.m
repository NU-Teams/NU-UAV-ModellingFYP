function [V, alpha, beta] = AeroAngles2(X_k)

alpha = X_k(3);
beta = X_k(2);

u = X_k(1);

V = u/(cos(alpha)*cos(beta));

end