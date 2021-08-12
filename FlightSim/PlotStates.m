function Figures = PlotStates(T, X_nl, X_lin, AXIS, config_str, manoeuvre_str)


if AXIS == "long"
    
    figure_str = 'Long. States';
    title_str = [figure_str ': ' manoeuvre_str];
    subtitle_str = config_str;
    
    legend1 = {'u nl'; 'u lin'};
    
    legend2 = {'\alpha nl'; '\alpha lin'};
    alpha_nl = atan(X_nl(2,:)./X_nl(1,:));
    alpha_lin = atan(X_lin(2,:)./X_lin(1,:));
    
    X_nl(2,:) = alpha_nl*180/pi;
    X_lin(2,:) = alpha_lin*180/pi;
    
    legend3 = {'q nl'; 'q lin'};
    X_nl(3,:) = X_nl(3,:)*180/pi;
    X_lin(3,:) = X_lin(3,:)*180/pi;
    
    legend4 = {'\theta nl'; '\theta lin'};
    X_nl(4,:) = X_nl(4,:)*180/pi;
    X_lin(4,:) = X_lin(4,:)*180/pi;
    
    legend5 = {'z_e nl'; 'z_e lin'};
    
elseif AXIS == "lat"
    figure_str = 'Lat. States';
    title_str = [figure_str ': ' manoeuvre_str];
    subtitle_str = config_str;
    
    legend1 = {'\beta nl'; '\beta lin'};
    X_nl(1,:) = X_nl(1,:)*180/pi;
    X_lin(1,:) = X_lin(1,:)*180/pi;
    legend2 = {'p nl'; 'p lin'};
    X_nl(2,:) = X_nl(2,:)*180/pi;
    X_lin(2,:) = X_lin(2,:)*180/pi;
    legend3 = {'r nl'; 'r lin'};
    
    X_nl(3,:) = X_nl(3,:)*180/pi;
    X_lin(3,:) = X_lin(3,:)*180/pi;
    
    legend4 = {'\phi nl'; '\phi lin'};
    X_nl(4,:) = X_nl(4,:)*180/pi;
    X_lin(4,:) = X_lin(4,:)*180/pi;
    
    legend5 = {'\psi nl'; '\psi lin'};
    X_nl(5,:) = X_nl(5,:)*180/pi;
    X_lin(5,:) = X_lin(5,:)*180/pi;
    
else
    disp('PlotStates: error')
    return
end


Figures(1) = figure('Name', figure_str);
subplot(5,1,1)
title(title_str, subtitle_str)
    hold on
    grid on
    plot(T, X_nl(1,:),'LineWidth', 1.5)
    plot(T, X_lin(1,:),'LineWidth', 1.5)
    legend(legend1)
    hold off
subplot(5,1,2)
    hold on
    grid on
    plot(T, X_nl(2,:),'LineWidth', 1.5)
    plot(T, X_lin(2,:),'LineWidth', 1.5)
    legend(legend2)
    hold off
subplot(5,1,3)
    hold on
    grid on
    plot(T, X_nl(3,:),'LineWidth', 1.5)
    plot(T, X_lin(3,:),'LineWidth', 1.5)
    legend(legend3)
    hold off
subplot(5,1,4)
    hold on
    grid on
    plot(T, X_nl(4,:),'LineWidth', 1.5)
    plot(T, X_lin(4,:),'LineWidth', 1.5)
    legend(legend4)
    hold off
subplot(5,1,5)
    hold on
    grid on
    plot(T, X_nl(5,:),'LineWidth', 1.5)
    plot(T, X_lin(5,:),'LineWidth', 1.5)
    legend(legend5)
    hold off