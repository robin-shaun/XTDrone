
# Period values
T = [1, 1, 2, 4, 6, 8, 8, 1];
# Gain values
G = [0, 1, 1, 0.5, 0.375, 0.375, 0, 0];

figure(1)
clf()
plot(T,G,'r-o')
fill(T,G,'g')
xlabel('P-M Peak Period ($T_p$) [s]')
ylabel('Gain ($\gamma$) [m/m]')
grid(True)
xlim([0, 9])
ylim([0, 1.1])
title('P-M Wavefield Model Envelope')
show()
