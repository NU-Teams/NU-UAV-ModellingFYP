function C_be = earth2body(CHI)

phi     = CHI(1);
theta   = CHI(2);
psi     = CHI(3);

C_be    = C_x(phi)*C_y(theta)*C_(psi);
end