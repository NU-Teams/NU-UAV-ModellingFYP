%PLOT Argand Diagram and Time Domain Response for analysis
function argandDiagrams = PlotStability(T, AIRCRAFT)


for k = 1:2
    
    if k == 1
        A = AIRCRAFT.Long.A;
    elseif k == 2
        A = AIRCRAFT.Lat.A;
    end

    [V, E] = eig(A);
    
    for coln = 1:5

        for row = 1:5

            lambda = E(coln, coln);
            ic = V(row,coln);
            lambdax = [0, real(ic)];
            lambday = [0, imag(ic)];

            result = ic*exp(lambda.*T);

            argandDiagrams(row, coln, k).lambda = [lambdax', lambday', 0*lambdax'];
            argandDiagrams(row, coln, k).result = [real(result)', imag(result)' T'];
            argandDiagrams(row, coln, k).lambdaEnd = [lambdax(2), lambday(2), 0*lambdax(2)];

        end
    end
end

end

