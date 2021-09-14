function Figures = PlotData(T, Xlin, Xnl)

u = [Xlin(1,:); Xnl(1,:)];
v = [Xlin(2,:); Xnl(2,:)];
w = [Xlin(3,:); Xnl(3,:)];
p = [Xlin(4,:); Xnl(4,:)];
q = [Xlin(5,:); Xnl(5,:)];
r = [Xlin(6,:); Xnl(6,:)];
q0 = [Xlin(7,:); Xnl(7,:)];
q1 = [Xlin(8,:); Xnl(8,:)];
q2 = [Xlin(9,:); Xnl(9,:)];
q3 = [Xlin(10,:); Xnl(10,:)];
x = [Xlin(11,:); Xnl(11,:)];
y = [Xlin(12,:); Xnl(12,:)];
z = [Xlin(13,:); Xnl(13,:)];


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
        plot(T, x(1,:), 'LineWidth', 1.5)
        plot(T, x(2,:),'--', 'LineWidth', 1.5)
        ylabel('x')
        hold off
    subplot(7, 1, 7)
        hold on
        grid on
        plot(T, z, 'LineWidth', 1.5)
        ylabel('z')
        hold off

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

end