close all;
clear all;
A = [- 0.322 -1.12;
     6.87    -0.32];
 
B = [0;0];

poles = eig(A)

figure(1);
plot(poles,'o'); grid;

a = real(poles);
b = imag(poles);

zeta_d = -a/((b.^2 + a.^2).^(1/2));
omega_n = (b.^2 + a.^2).^(1/2);



