function CHI = Q2E(X)
% Jason Iredale, 30/04/2021
% Note: Jason provided his assignment 1 code after comparison with Inga,
% Marty and Ash where all 4 individual codes were the same.
% quat2EA is a simple quaternion to euler-angle transformation.

q0 = X(7,:);
q1 = X(8,:);
q2 = X(9,:);
q3 = X(10,:);

phi   = atan2((q2.*q3+q0.*q1),(q0.^2+q3.^2-0.5));
theta = atan2((q0.*q2-q1.*q3),sqrt((q0.^2+q1.^2-0.5).^2+(q1.*q2+q0.*q3).^2));
psi   = atan2((q1.*q2+q0.*q3),(q0.^2+q1.^2-0.5));


CHI = [phi; theta; psi];

end
