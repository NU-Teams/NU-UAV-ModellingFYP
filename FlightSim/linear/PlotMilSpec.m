function [Figures, dn_dalpha] = PlotMilSpec(FD, Vt)


[V,E,Wn,zeta] = eig_analysis(FD.Long.A);


Figures(1) = figure('Name','Milspec1');
title_str = ['MilSpec: Short Period'];
title(title_str)
xlabel(' Damping Ratio \zeta SP')
ylabel(' Nat. Frequency \omega_n SP')
hold on
grid on
plot(zeta(1), Wn(1), 'or','LineWidth', 1.5')
plot(zeta(2), Wn(2), 'xm','LineWidth', 1.5')
hold off


Q = 0.5*1.206*Vt^2;
dn_dalpha = FD.Aero.CLa*Q*FD.Geom.S/(FD.Inertia.m*FD.Inertia.g);

figure('Name','Milspec2');
loglog(logspace(0,2), logspace(-0.523,0.447), 'k', 'LineWIdth', 1.5);
title_str = ['MilSpec: Short Period'];
title(title_str)
xlabel(' n/\alpha [g/rad]')
ylabel(' Nat. Frequency \omega_n SP')
hold on
loglog(logspace(0,2), logspace(0.301,1.301), 'k', 'LineWIdth', 1.5);
loglog(dn_dalpha, Wn(1), 'or','LineWidth', 1.5')
loglog(dn_dalpha, Wn(2), 'xm','LineWidth', 1.5')
grid on
hold off

end