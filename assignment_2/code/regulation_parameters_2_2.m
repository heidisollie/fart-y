close all;
clear all;
clc;

parameters_longitudal_autopilot;

s = tf('s');
sys = ss(A,B,C,D);
Phi = inv(s*eye(5)-A);
trans = C*Phi*B + D;


plant = -0.65/(s - 2.)*(omega_n_phi)^2/(s^2 + 2*zeta_phi*omega_n_phi*s + omega_n_phi^2);

H_evCar = s*(s^2+(a_phi_1 + a_phi_2*k_d_phi)*s + a_phi_2*k_p_phi);

controlSystemDesigner('rlocus',1/H_evCar);



H_chi = (2*zeta_chi*omega_n_chi*s + omega_n_chi^2)/(s^2 + 2*zeta_chi*omega_n_chi*s + omega_n_chi^2);

%controlSystemDesigner('rlocus',H_chi);

k_p_chi = 2*zeta_chi*omega_n_chi*V_g/g;

k_i_chi = omega_n_chi^2*V_g/g;

%% Task 2d

H_phi = a_phi_2*(k_p_phi*s + k_i_phi)/(s^2*(s + a_phi_1 + a_phi_2*k_d_phi));

H_chi = g/V_g*(a_phi_2*(k_p_phi*s + k_i_phi)*(k_i_chi+k_p_chi*s))/(s^4*(s+a_phi_1 + a_phi_2*k_d_phi));

figNum = 1;
figure(figNum)
bode(H_phi); hold on; grid on;
bode(H_chi); legend('H_{\phi}','H_{\chi}');

figNum = figNum +1;
figure(figNum)
margin(H_phi);grid on; legend('H_{\phi}');

figNum = figNum +1;
figure(figNum)
margin(H_chi);grid on; legend('H_{\chi}');

% Changing the freq

omega_n_chi = omega_n_phi;

k_p_chi = 2*zeta_chi*omega_n_chi*V_g/g;

k_i_chi = omega_n_chi^2*V_g/g;

H_phi = a_phi_2*(k_p_phi*s + k_i_phi)/(s^2*(s + a_phi_1 + a_phi_2*k_d_phi));

H_chi = g/V_g*(a_phi_2*(k_p_phi*s + k_i_phi)*(k_i_chi+k_p_chi*s))/(s^4*(s+a_phi_1 + a_phi_2*k_d_phi));

figNum = figNum +1;
figure(figNum)
bode(H_phi); hold on; grid on;
bode(H_chi); legend('H_{\phi}','H_{\chi}');

figNum = figNum +1;
figure(figNum)
margin(H_phi);grid on; legend('H_{\phi}');

figNum = figNum +1;
figure(figNum)
margin(H_chi);grid on; legend('H_{\chi}');







