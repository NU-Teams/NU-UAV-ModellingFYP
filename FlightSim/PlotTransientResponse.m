function Figures = PlotTransientResponse(T, Xlin, Xnl, U)
% Units
deg = 180/pi;   % rad to deg
kts = 1.944;    % m/s to knots
ft = 3.2808399; % m to ft

%% Unpack Input
u   = [Xlin(1,:); Xnl(1,:)];
v   = [Xlin(2,:); Xnl(2,:)];
w   = [Xlin(3,:); Xnl(3,:)];
p   = [Xlin(4,:); Xnl(4,:)];
q   = [Xlin(5,:); Xnl(5,:)];
r   = [Xlin(6,:); Xnl(6,:)];
q0  = [Xlin(7,:); Xnl(7,:)];
q1  = [Xlin(8,:); Xnl(8,:)];
q2  = [Xlin(9,:); Xnl(9,:)];
q3  = [Xlin(10,:); Xnl(10,:)];
x   = [Xlin(11,:); Xnl(11,:)];
y   = [Xlin(12,:); Xnl(12,:)];
z   = [Xlin(13,:); Xnl(13,:)];

VT      = [Xlin(14,:); Xnl(14,:)]*kts;
beta    = [Xlin(15,:); Xnl(15,:)]*deg;
alpha   = [Xlin(16,:); Xnl(16,:)]*deg;

phi     = [Xlin(17,:); Xnl(17,:)]*deg;
theta   = [Xlin(18,:); Xnl(18,:)]*deg;
psi     = [Xlin(19,:); Xnl(19,:)]*deg;

delta_T = U(1,:);
delta_e = U(2,:)*deg;
delta_a = U(3,:)*deg;
delta_r = U(4,:)*deg;


%% Longitudinal Transient Response
Figures(1) = figure('Name', 'Longitudinal Transient');
    subplot(7, 1, 1)
        hold on
        grid on
        plot(T, u, 'LineWidth', 1.5)
        ylabel('u')
        hold off
    subplot(7, 1, 2)
        hold on
        grid on
        plot(T, w, 'LineWidth', 1.5)
        ylabel('w')
        hold off
    subplot(7, 1, 3)
        hold on
        grid on
        plot(T, q, 'LineWidth', 1.5)
        ylabel('q')
        hold off
    subplot(7, 1, 4)
        hold on
        grid on
        plot(T, q0, 'LineWidth', 1.5)
        ylabel('q_0')
        hold off
    subplot(7, 1, 5)
        hold on
        grid on
        plot(T, q2, 'LineWidth', 1.5)
        ylabel('q_2')
        hold off
    subplot(7, 1, 6)
        hold on
        grid on
        plot(T, x, 'LineWidth', 1.5)
        ylabel('x')
        hold off
    subplot(7, 1, 7)
        hold on
        grid on
        plot(T, z, 'LineWidth', 1.5)
        ylabel('z')
        hold off


%% Lateral Transient Response
Figures(2) = figure('Name', 'Lateral Transient');
    subplot(6, 1, 1)
        hold on
        grid on
        plot(T, v, 'LineWidth', 1.5)
        ylabel('v')
        hold off
    subplot(6, 1, 2)
        hold on
        grid on
        plot(T, p, 'LineWidth', 1.5)
        ylabel('p')
        hold off
    subplot(6, 1, 3)
        hold on
        grid on
        plot(T, r, 'LineWidth', 1.5)
        ylabel('r')
        hold off
    subplot(6, 1, 4)
        hold on
        grid on
        plot(T, q1, 'LineWidth', 1.5)
        ylabel('q_1')
        hold off
    subplot(6, 1, 5)
        hold on
        grid on
        plot(T, q3, 'LineWidth', 1.5)
        ylabel('q_3')
        hold off
    subplot(6, 1, 6)
        hold on
        grid on
        plot(T, y, 'LineWidth', 1.5)
        ylabel('y')
        hold off

        
%% AeroAngles Transient Response
Figures(3) = figure('Name','AeroAngles Transent');
    subplot(3, 1, 1)
        hold on
        grid on
        plot(T, VT, 'LineWidth', 1.5)
        ylabel('V_T [kts]')
        hold off
    subplot(3, 1, 2)
        hold on
        grid on
        plot(T, beta, 'LineWidth', 1.5)
        ylabel('\beta [deg]')
        hold off
    subplot(3, 1, 3)
        hold on
        grid on
        plot(T, alpha, 'LineWidth', 1.5)
        ylabel('\alpha [deg]')
        hold off

%% Euler Angles

Figures(3) = figure('Name','Euler Angles Transent');
    subplot(3, 1, 1)
        hold on
        grid on
        plot(T, phi, 'LineWidth', 1.5)
        ylabel('\phi [deg]')
        hold off
    subplot(3, 1, 2)
        hold on
        grid on
        plot(T, theta, 'LineWidth', 1.5)
        ylabel('\theta [deg]')
        hold off
    subplot(3, 1, 3)
        hold on
        grid on
        plot(T, psi, 'LineWidth', 1.5)
        ylabel('\psi [deg]')
        hold off
        
%% Controls

Figures(4) = figure('Name','Controls Transent');
    subplot(4, 1, 1)
        hold on
        grid on
        plot(T, delta_T, 'LineWidth', 1.5)
        ylabel('\delta_T')
        hold off
    subplot(4, 1, 2)
        hold on
        grid on
        plot(T, delta_e, 'LineWidth', 1.5)
        ylabel('\delta_e [deg]')
        hold off
    subplot(4, 1, 3)
        hold on
        grid on
        plot(T, delta_a, 'LineWidth', 1.5)
        ylabel('\delta_a [deg]')
        hold off
    subplot(4, 1, 4)
        hold on
        grid on
        plot(T, delta_r, 'LineWidth', 1.5)
        ylabel('\delta_r [deg]')
        hold off
end