% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

V_g = 637*3.6; % m/s
delta_a_max = 25*rad2deg;
e_phi_max = 15*rad2deg;
zeta_phi = 0.707;
a_phi_1 = 36.67; %2.87;
a_phi_2 = 211.35; %-0.65;

omega_n_phi = sqrt(abs(a_phi_2)*delta_a_max /e_phi_max);

k_p_phi = delta_a_max /e_phi_max;
k_d_phi = (2*zeta_phi*omega_n_phi - a_phi_1) / a_phi_2;
k_i_phi = 1; %temp
% find root lokus k_p_phi = 
%k_p_chi = 2*zeta_chi*omega_n_chi*V_g/g;
%k_i_chi = omega_n_chi^2*Vg/g;

A = [ -0.322 0.052 0.028 -1.12 0.002;
     0 0 1 -0.001 0;
     -10.6 0 -2.87 0.46 -0.65;
     6.87 0 -0.04 -0.32 -0.02;
     0 0 0 0 -10];
B = [ 0 0 0 0 10]';
C = [0 0 0 1 0;
     0 0 1 0 0;
     1 0 0 0 0 ;
     0 1 0 0 0];
D = [0 0 0 0]';

s = tf('s');
sys = ss(A,B,C,D);
Phi = inv(s*eye(5)-A);
trans = C*Phi*B + D;


plant = -0.65/(s - 2.)*(omega_n_phi)^2/(s^2 + 2*zeta_phi*omega_n_phi*s + omega_n_phi^2);

H_el = (a_phi_2*(k_i_phi/s + k_p_phi))/((1+k_d_phi*(a_phi_2/(s+a_phi_1)))*(s+a_phi_1)*s+a_phi_2*(k_i_phi/s+k_p_phi));

H_bok = (a_phi_2*k_p_phi*(s+(k_i_phi + k_p_phi)))/(s^3 + (a_phi_1 + a_phi_2*k_d_phi)*s^2 + a_phi_2*k_p_phi*s + a_phi_2*k_i_phi);

H_car = (s^3 + (a_phi_1 + a_phi_2*k_d_phi)*s^2 + a_phi_2*k_p_phi*s + a_phi_2*k_i_phi);

H_evan = 1 + k_i_phi*(a_phi_2/(s*(s^2+(a_phi_1 + a_phi_2*k_d_phi)*s + a_phi_2*k_p_phi)));

H_evCar = s*(s^2+(a_phi_1 + a_phi_2*k_d_phi)*s + a_phi_2*k_p_phi);

controlSystemDesigner('rlocus',H_evCar);



