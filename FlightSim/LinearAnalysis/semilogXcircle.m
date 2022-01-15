function circ = semilogXcircle(r, centre, thickness, m)

x = logspace(-1, 1, 1000);

for k = 1:length(x)
    y_circ(k) = thickness*sqrt(r^2 - (log10(x(k))-centre(1)).^2) + centre(2);
end

x_top = x(imag(y_circ)==0);
y_top = y_circ(imag(y_circ)==0);

x = [x_top  flip(x_top)];
y = [y_top -flip(y_top)+2*centre(2)];

line_y = m*log10(x); 

circ = [x' (line_y+y)'];
circ(end,:) = circ(1,:);

end