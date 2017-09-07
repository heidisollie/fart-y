% problem 2.3

deg2rad = pi/180;   
rad2deg = 180/pi;

phi = 0 *deg2rad;
theta = 2.0 *deg2rad;
psi = 30 * deg2rad;
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;


R_n_b = Rzyx(phi,theta,psi);
v_n_c = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)];
     
v_b_c = inv(R_n_b)*v_n_c

     