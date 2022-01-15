function [E, V,D, Wn,zeta] = eig_analysis(A)
% Eigen Analysis 
% Inga, Marty 27/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    E           = eig(A);   % column vector of eigen values
    [V, D]      = eig(A);   % eigen vectors and diagonal of eigen values : A* = V*D*(V^-1)
    [Wn, zeta]  = damp(A);  % natrural angular frequency and damping factor
    
end

