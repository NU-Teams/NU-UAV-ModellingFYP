function Figures = TimeSeriesPlots(T, X_lat, X_long, config_str, manoeuvre_str)
%%% Marty Shannon & Inga Leinasars, 31/05/2021 

figure_str = 'State Variables';
title_str = [figure_str ': ' manoeuvre_str];
subtitle_str = config_str;

% beta & alpha 
    Figures(1) = figure('Name', figure_str); 
    title(title_str, subtitle_str);
    hold on
    grid minor
    %plot(T, X_long(2,:)*180/pi,'LineWidth', 1.5)
    plot(T, X_lat(1,:)*180/pi,'LineWidth', 1.5)
    legend('\beta')
    xlabel('time [s]')
    ylabel('angle [degs]')
    hold off
% rates 
Figures(2) = figure('Name', figure_str);     
    hold on
    grid minor
    plot(T, X_lat(2,:)*180/pi,'LineWidth', 1.5)
    %plot(T, X_long(3,:)*180/pi,'LineWidth', 1.5)
    plot(T, X_lat(3,:)*180/pi,'LineWidth', 1.5)
    legend('p', 'r')
    xlabel('time [s]')
    ylabel('rate [deg/s]')
    hold off
% angles
Figures(3) = figure('Name', figure_str);
    hold on
    grid minor
    plot(T, X_lat(4,:)*180/pi,'LineWidth', 1.5)
    %plot(T, X_long(4,:)*180/pi,'LineWidth', 1.5)
    plot(T, X_lat(5,:)*180/pi,'LineWidth', 1.5)
    legend('\phi', '\psi')
    xlabel('time [s]')
    ylabel('angle [degs]')
    hold off
% u 
%{
Figures(4) = figure('Name', figure_str); 
    hold on
    grid minor
    plot(T, X_long(1,:)*1.944,'LineWidth', 1.5)
    legend('u')
    xlabel('time [s]')
    ylabel('airspeed [knots]')
    hold off
% z_e 
Figures(5) = figure('Name', figure_str); 
    hold on
    grid minor
    plot(T, -X_long(5,:)*3.2808399,'LineWidth', 1.5)
    legend('z_e')
    xlabel('time [s]')
    ylabel('altitude [ft]')
    hold off
%}
end 