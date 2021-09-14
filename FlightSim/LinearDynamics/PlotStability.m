function [] = PlotStability(T, FD, AXIS)
%PLOT Argand Diagram and Time Domain Response for analysis
%   Marty     27/05/2021


    if AXIS == "lat"
        A = FD.Lat.A;
    elseif AXIS == "long"
        A = FD.Long.A;
    end
    
    [V, E] = eig(A);
    [Wn, zeta] = damp(A);
    
%% Argand Diagram


colour = ['r', 'm', 'g', 'b', 'k'];
colour2 = ['--r', '--m', '--g', '--b', '--k'];
colour3 = ['or', 'om', 'og', 'ob', 'ok'];
    
%set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
% ax = gca;
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = 'origin';


for coln = (1:5)
    figure('Name','Argand Diagram')
%     subplot(2,1,1)
    title_str = [AXIS ' Argand Diagram ' num2str(coln)];
    subtitle_str = ['\omega_n: ' num2str(Wn(coln)) ' \zeta: ' num2str(zeta(coln))];
    title(title_str, subtitle_str)
    xlabel('Real Axis')
    ylabel('Imaginary Axis')
    zlabel('Time')
    hold on
    grid on
    
    max_real_range = max(abs(real(V(:,coln))));
    max_imag_range = max(abs(imag(V(:,coln))));
    max_range = max([max_real_range, max_imag_range]);
    
    xlim([-max_range, max_range]);
    ylim([-max_range, max_range]);
    
    for row = (1:5)

        lambda = E(coln, coln);
        ic = V(row,coln);
        lambdax = [0, real(ic)];
        lambday = [0, imag(ic)];

        result = ic*exp(lambda.*T);
        
        plot3(lambdax, lambday, 0*lambdax, colour(row),'LineWidth', 1);
        plot3(real(result), imag(result), T, colour2(3*row-2:3*row), 'LineWidth', 1)
        set(gca, 'ZDir','reverse')
        plot3(lambdax(2), lambday(2), 0*lambdax(2), colour3(2*row-1:2*row));
    end
    hold off
    
%     subplot(2,1,2)
%     
%     xlabel('Time')
%     ylabel('Real Axis')
%     hold on
%     grid on
%     for row = [1:5]
%         result_real = real(result);
%         plot(T, result_real, colour(row), 'LineWidth', 1.5)
%     end
end


end

