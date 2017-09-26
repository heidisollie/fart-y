% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

V_g = 637*3.6; % m/s
delta_a_max = 25*rad2deg;
e_phi_max = 15*rad2deg;
zeta_phi = 0.707;
a_phi_1 = 2.87;
a_phi_2 = -0.65;

omega_n_phi = sqrt(abs(a_phi_2)*delta_a_max /e_phi_max);

k_p_phi = delta_a_max /e_phi_max;
k_d_phi = (2*zeta_phi*omega_n_phi - a_phi_1) / a_phi_2;
% find root lokus k_p_phi = 
%k_p_chi = 2*zeta_chi*omega_n_chi*V_g/g;
%k_i_chi = omega_n_chi^2*Vg/g;

s = tf('s');
plant = -0.65/(s - 2.)*(omega_n_phi)^2/(s^2 + 2*zeta_phi*omega_n_phi*s + omega_n_phi^2);
controlSystemDesigner('rlocus',plant);