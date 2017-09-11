% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% euler angles
phi = 0;
theta = 2.0*rad2deg;
psi = 30*rad2deg;

% current parameters
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

v_n_c_n = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned

v_b_b_c = [1.5;  0; 0];

R_n_b = Rzyx(phi,theta,psi); 

v_b_r = v_b_b_c - R_n_b'*v_n_c_n;
U_r = norm(v_b_r)

sideslip_angle = asin(v_b_r(2)/U_r)*rad2deg
