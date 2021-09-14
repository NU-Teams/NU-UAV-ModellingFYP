function [V,E,Wn,zeta] = eig_analysis(A)
% Eigen Analysis 
% Inga, Marty 27/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [V, E] = eig(A);
    [Wn, zeta] = damp(A);
end

