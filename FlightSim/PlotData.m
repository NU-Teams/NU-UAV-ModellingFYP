function Figures = PlotData(T, Xlin, Xnl)

Figures(1) = figure(1);
subplot(7, 1, 1)
hold on
plot(T, Xlin(:,1))
plot(T, Xnl(:,1),'--')
hold off
subplot(7, 1, 2)
hold on
plot(T, Xlin(:,3))
plot(T, Xnl(:,3),'--')
hold off
subplot(7, 1, 3)
hold on
plot(T, Xlin(:,5))
plot(T, Xnl(:,5))
hold off
subplot(7, 1, 4)
hold on
plot(T, Xlin(:,7))
plot(T, Xnl(:,7))
hold off
subplot(7, 1, 5)
hold on
plot(T, Xlin(:,9))
plot(T, Xnl(:,9))
hold off
subplot(7, 1, 6)
hold on
plot(T, Xlin(:,11))
plot(T, Xnl(:,11))
hold off
subplot(7, 1, 7)
hold on
plot(T, Xlin(:,13))
plot(T, Xnl(:,13))
hold off

Figures(2) = figure(2);
subplot(6, 1, 1)
hold on
plot(T, Xlin(:,2))
plot(T, Xnl(:,2),'--')
hold off
subplot(6, 1, 2)
hold on
plot(T, Xlin(:,4))
plot(T, Xnl(:,4),'--')
hold off
subplot(6, 1, 3)
hold on
plot(T, Xlin(:,6))
plot(T, Xnl(:,6))
hold off
subplot(6, 1, 4)
hold on
plot(T, Xlin(:,8))
plot(T, Xnl(:,8))
hold off
subplot(6, 1, 5)
hold on
plot(T, Xlin(:,10))
plot(T, Xnl(:,10))
hold off
subplot(6, 1, 6)
hold on
plot(T, Xlin(:,12))
plot(T, Xnl(:,12))
hold off