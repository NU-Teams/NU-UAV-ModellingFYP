
% level 1

lvl(1).lwr.p1 = [90 5];
lvl(1).lwr.p2 = [3 0.9];

lvl(1).upr.p1 = [10 6];
lvl(1).upr.p2 = [100 19];
    
% level 2

lvl(2).lwr.p1 = [5 0.9];
lvl(2).lwr.p2 = [100 4];

lvl(2).upr.p1 = [5  7];
lvl(2).upr.p2 = [40 20];

% level 3

lvl(3).lwr.p1 = [5 0.9];
lvl(3).lwr.p2 = [100 4];

lvl(3).upr.p1 = [0 0];
lvl(3).upr.p2 = [0 0];

%{
    % level 1

    lvl(1).lwr.p1 = [5 0.9];
    lvl(1).lwr.p2 = [100 4];

    lvl(1).upr.p1 = [7  5];
    lvl(1).upr.p2 = [10 6];

    % level 2

    lvl(2).lwr.p1 = [5 0.7];
    lvl(2).lwr.p2 = [100 3];

    lvl(2).upr.p1 = [5  7];
    lvl(2).upr.p2 = [10 10];

    % level 3

    lvl(3).lwr.p1 = [5 0.7];
    lvl(3).lwr.p2 = [100 3];

    lvl(3).upr.p1 = [0 0];
    lvl(3).upr.p2 = [0 0];
%}

base = 10;

for j = 1:3

    points_lwr = lvl(j).lwr;
    points_upr = lvl(j).upr;
    
    [lvl(j).lwr.k, lvl(j).lwr.n, lvl(j).lwr.func] = points2loglogline(points_lwr, base);
    [lvl(j).upr.k, lvl(j).upr.n, lvl(j).upr.func] = points2loglogline(points_upr, base);

end


x = 1:100;

figure(1)

for j = 1:3
    
    loglog(x, lvl(j).lwr.func(x))
    hold on
    loglog(x, lvl(j).upr.func(x))
    
end

grid on
hold off




function [k, n, func] = points2loglogline(points, base)

% unpack
p1 = log(points.p1)/log(base);
p2 = log(points.p2)/log(base);

grad = (p2(2)-p1(2)) / (p2(1)-p1(1));
%grad = (log(p2(2)/p1(2))) / (log(p2(1)/p1(1)));
y_int = p2(2)-grad*p2(1);

n = grad;
k = base^y_int;

func = @(x) k*(x.^n);
end