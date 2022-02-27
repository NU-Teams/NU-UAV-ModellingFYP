%PLOT Argand Diagram and Time Domain Response for analysis
function argandDiagrams = PlotStability(T, AIRCRAFT)

A = AIRCRAFT.A;

[U, LAMBDA] = eig(A);

for coln = 1:12

    for row = 1:12

        lambda = LAMBDA(coln, coln);
        initCond = U(row,coln);
        lambdax = [0, real(initCond)];
        lambday = [0, imag(initCond)];

        result = initCond*exp(lambda.*T);

        argandDiagrams(row, coln).lambda	= [lambdax',        lambday',       0*lambdax'];
        argandDiagrams(row, coln).result	= [real(result)',	imag(result)',  T'];
        argandDiagrams(row, coln).lambdaEnd = [lambdax(2),      lambday(2),     0*lambdax(2)];

    end
end

end

