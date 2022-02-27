%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% E2Q is an euler-angle to quaternion transformation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Quaternion = E2Q(CHI)

    phi_2	= CHI(1,:)./2;
    theta_2 = CHI(2,:)./2;
    psi_2	= CHI(3,:)./2;

    q0 = cos(phi_2).*cos(theta_2).*cos(psi_2) + sin(phi_2).*sin(theta_2).*sin(psi_2);
    q1 = sin(phi_2).*cos(theta_2).*cos(psi_2) - cos(phi_2).*sin(theta_2).*sin(psi_2);
    q2 = cos(phi_2).*sin(theta_2).*cos(psi_2) + sin(phi_2).*cos(theta_2).*sin(psi_2);
    q3 = cos(phi_2).*cos(theta_2).*sin(psi_2) - sin(phi_2).*sin(theta_2).*cos(psi_2);

    magnitude = norm([q0; q1; q2; q3]);

    Quaternion = [q0;q1;q2;q3]/magnitude;

end